import type { SegmentId, Vec3 } from "../core/types";
import { clampLength } from "./math";

/** A copied, symmetric Rapier world inverse-inertia tensor; no WASM-backed views. */
export interface RecoveryBodyInverseInertia {
  readonly m11: number; readonly m12: number; readonly m13: number;
  readonly m22: number; readonly m23: number; readonly m33: number;
}

/** Error and relative angular velocity are expressed in the same world frame. */
export interface RecoveryMotorIntent {
  readonly id: SegmentId;
  readonly parent: SegmentId | null;
  readonly error: Vec3;
  readonly velocity: Vec3;
  readonly kp: number;
  readonly kd: number;
  readonly feedforward: Vec3;
  readonly cap: number;
}

export interface RecoveryMotorSolveOptions {
  /** Equilibrium torque cancels an external load; command torque accelerates the body. */
  readonly feedforwardMode?: "equilibrium" | "command";
}

function solvePositiveDefinite(matrix: Float64Array, rhs: Float64Array): Float64Array {
  const size = rhs.length;
  // In-place lower Cholesky factor. Positive diagonal gain terms make the
  // symmetric response positive definite, even for a locked inertia axis.
  for (let row = 0; row < size; row++) {
    for (let column = 0; column <= row; column++) {
      let value = matrix[row * size + column];
      for (let inner = 0; inner < column; inner++) value -= matrix[row * size + inner] * matrix[column * size + inner];
      if (row === column) {
        if (!(value > 0) || !Number.isFinite(value)) throw new RangeError("Recovery motor response must be positive definite.");
        matrix[row * size + column] = Math.sqrt(value);
      } else matrix[row * size + column] = value / matrix[column * size + column];
    }
  }
  const result = new Float64Array(rhs);
  for (let row = 0; row < size; row++) {
    for (let column = 0; column < row; column++) result[row] -= matrix[row * size + column] * result[column];
    result[row] /= matrix[row * size + row];
  }
  for (let row = size - 1; row >= 0; row--) {
    for (let column = row + 1; column < size; column++) result[row] -= matrix[column * size + row] * result[column];
    result[row] /= matrix[row * size + row];
  }
  return result;
}

/** Exact three-axis block optimum with a Euclidean torque ceiling. */
function boundedBlock(a:number,b:number,c:number,d:number,e:number,f:number,r:Vec3,cap:number):Vec3 {
  if (cap === 0) return {x:0,y:0,z:0};
  const solve=(lambda:number):Vec3=>{
    const x=a+lambda,y=d+lambda,z=f+lambda;
    const A=y*z-e*e,B=c*e-b*z,C=b*e-c*y,D=x*z-c*c,E=b*c-x*e,F=x*y-b*b;
    const det=x*A+b*B+c*C;
    return {x:(A*r.x+B*r.y+C*r.z)/det,y:(B*r.x+D*r.y+E*r.z)/det,z:(C*r.x+E*r.y+F*r.z)/det};
  };
  const free=solve(0);
  if(Math.hypot(free.x,free.y,free.z)<=cap) return free;
  // ||(H+lambda I)^-1 r|| <= ||r||/lambda; this bounds the unique multiplier.
  let low=0,high=Math.hypot(r.x,r.y,r.z)/cap;
  for(let iteration=0;iteration<32;iteration++) {
    const middle=(low+high)*.5, value=solve(middle);
    if(Math.hypot(value.x,value.y,value.z)>cap) low=middle; else high=middle;
  }
  return clampLength(solve(high),cap);
}

/** Convex block Gauss-Seidel propagates every active ceiling to its neighbors. */
function solveBoundedPositiveDefinite(matrix:Float64Array,rhs:Float64Array,caps:readonly number[]):Float64Array {
  const size=rhs.length,result=solvePositiveDefinite(new Float64Array(matrix),rhs);
  let bounded=false;
  for(let block=0;block<caps.length;block++) {
    const row=block*3,free={x:result[row],y:result[row+1],z:result[row+2]};
    if(Math.hypot(free.x,free.y,free.z)>caps[block]) {
      const feasible=clampLength(free,caps[block]);
      result[row]=feasible.x;result[row+1]=feasible.y;result[row+2]=feasible.z;bounded=true;
    }
  }
  if(!bounded) return result;
  for(let iteration=0;iteration<96;iteration++) {
    let change=0;
    for(let block=0;block<caps.length;block++) {
      const row=block*3,local={x:rhs[row],y:rhs[row+1],z:rhs[row+2]};
      for(let column=0;column<size;column++) if(column<row||column>row+2) {
        local.x-=matrix[row*size+column]*result[column];
        local.y-=matrix[(row+1)*size+column]*result[column];
        local.z-=matrix[(row+2)*size+column]*result[column];
      }
      const value=boundedBlock(matrix[row*size+row],matrix[row*size+row+1],matrix[row*size+row+2],
        matrix[(row+1)*size+row+1],matrix[(row+1)*size+row+2],matrix[(row+2)*size+row+2],local,caps[block]);
      change=Math.max(change,Math.hypot(value.x-result[row],value.y-result[row+1],value.z-result[row+2]));
      result[row]=value.x;result[row+1]=value.y;result[row+2]=value.z;
    }
    if(change<1e-8) break;
  }
  return result;
}
/**
 * Solve all angular muscles from one measured state before applying any impulse.
 *
 * J has +I for each child and -I for its parent, so M = J I^-1 J^T includes
 * the response of every shared body. With K_j = kd_j dt + kp_j dt², the
 * semi-implicit motor equation is (I + K M) tau = kp error - kd velocity + FF.
 * Scaling each row by 1/K gives the symmetric positive-definite M + K^-1.
 *
 * Equilibrium FF assumes an opposing external acceleration -M FF. Including
 * that prediction adds (I + K M) FF to the RHS, equivalent to solving feedback
 * and adding FF once. Command FF instead belongs in the ordinary RHS.
 *
 * The caller applies each returned torque equally and oppositely. Per-muscle
 * ceilings constrain the simultaneous quadratic solve, including the FF offset;
 * clipping one free answer cannot leave neighboring motors predicting lost torque.
 * No body state is changed.
 */
export function solveRecoveryMotorTorques(
  motors: readonly RecoveryMotorIntent[],
  bodyInverseInertia: ReadonlyMap<SegmentId, RecoveryBodyInverseInertia>,
  dt: number,
  options: RecoveryMotorSolveOptions = {},
): Map<SegmentId, Vec3> {
  if (!(dt > 0) || !Number.isFinite(dt)) throw new RangeError("Recovery motor timestep must be positive and finite.");
  const size = motors.length * 3;
  const response = new Float64Array(size * size);
  const base = new Float64Array(size), feedforward = new Float64Array(size), gain = new Float64Array(size);
  const incidence = new Map<SegmentId, { index: number; sign: number }[]>();
  const seen = new Set<SegmentId>();
  const command = options.feedforwardMode === "command";
  for (const [index, motor] of motors.entries()) {
    if (seen.has(motor.id) || motor.id === motor.parent) throw new RangeError("Recovery motors require distinct child IDs and parents.");
    seen.add(motor.id);
    const values = [motor.kp, motor.kd, motor.cap, motor.error.x, motor.error.y, motor.error.z,
      motor.velocity.x, motor.velocity.y, motor.velocity.z, motor.feedforward.x, motor.feedforward.y, motor.feedforward.z];
    if (!values.every(Number.isFinite) || motor.kp < 0 || motor.kd < 0 || motor.cap < 0) throw new RangeError("Recovery motor inputs must be finite with nonnegative gains and caps.");
    const k = motor.kd * dt + motor.kp * dt * dt;
    for (const [axis, field] of (["x", "y", "z"] as const).entries()) {
      const row = index * 3 + axis;
      gain[row] = k;
      feedforward[row] = motor.feedforward[field];
      base[row] = motor.kp * motor.error[field] - motor.kd * motor.velocity[field] + (command ? feedforward[row] : 0);
    }
    for (const [id, sign] of [[motor.id, 1], [motor.parent, -1]] as const) {
      if (id === null) continue;
      const entries = incidence.get(id) ?? [];
      entries.push({ index, sign }); incidence.set(id, entries);
    }
  }
  for (const [body, entries] of incidence) {
    const inertia = bodyInverseInertia.get(body);
    if (!inertia || !Object.values(inertia).every(Number.isFinite)) throw new RangeError(`Missing or nonfinite recovery inertia for ${body}.`);
    const tensor = [inertia.m11, inertia.m12, inertia.m13, inertia.m12, inertia.m22, inertia.m23, inertia.m13, inertia.m23, inertia.m33];
    for (const a of entries) for (const b of entries) {
      for (let row = 0; row < 3; row++) for (let column = 0; column < 3; column++) {
        response[(a.index * 3 + row) * size + b.index * 3 + column] += a.sign * b.sign * tensor[row * 3 + column];
      }
    }
  }
  // Zero-gain rows are known torques, including pure feedforward motors. Their
  // acceleration still contributes to every coupled positive-gain row.
  const active: number[] = [], known: number[] = [];
  for (let row = 0; row < size; row++) (gain[row] > 0 ? active : known).push(row);
  const reducedSize = active.length;
  const matrix = new Float64Array(reducedSize * reducedSize), rhs = new Float64Array(reducedSize);
  const torque = new Float64Array(size);
  for (const [index,motor] of motors.entries()) if(gain[index*3]===0) {
    const value=clampLength(motor.feedforward,motor.cap);
    torque[index*3]=value.x;torque[index*3+1]=value.y;torque[index*3+2]=value.z;
  }
  for (let row = 0; row < reducedSize; row++) {
    const original = active[row];
    rhs[row] = base[original] / gain[original];
    for (const column of known) rhs[row] -= response[original * size + column] * (torque[column] - (command ? 0 : feedforward[column]));
    for (let column = 0; column < reducedSize; column++) matrix[row * reducedSize + column] = response[original * size + active[column]];
    matrix[row * reducedSize + row] += 1 / gain[original];
  }
  if (!command) for(let row=0;row<reducedSize;row++)
    for(let column=0;column<reducedSize;column++) rhs[row]+=matrix[row*reducedSize+column]*feedforward[active[column]];
  const caps=active.filter((_,index)=>index%3===0).map(row=>motors[Math.floor(row/3)].cap);
  const solved = solveBoundedPositiveDefinite(matrix, rhs, caps);
  for (let row = 0; row < reducedSize; row++) torque[active[row]] = solved[row];
  const result = new Map<SegmentId, Vec3>();
  for (const [index, motor] of motors.entries()) {
    const row = index * 3;
    result.set(motor.id, clampLength({
      x: torque[row],
      y: torque[row + 1],
      z: torque[row + 2],
    }, motor.cap));
  }
  return result;
}
