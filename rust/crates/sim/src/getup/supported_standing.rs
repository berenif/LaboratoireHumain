//! Upright-reference inverse statics, driven by measured foot contacts.
//!
//! Desired ground reactions are inputs to a joint-torque calculation only.
//! They are never applied to a body and never count as evidence of support.
use crate::*;

type SupportReactions = [(usize, Vector, Vector); 2];

/// A control intention bound to a recovered stance, never a body-state edit.
/// Every build decodes this type to refuse unsupported active checkpoints.
#[derive(Clone, Copy, Serialize, Deserialize)]
pub(super) struct StandingReference {
    pub(super) com: Vector,
    pub(super) forward: Vector,
    pub(super) lateral: Vector,
}

/// Algebraic evidence for admission/transition only. Production control remains
/// the unchanged source-PD-plus-compensation path below.
#[derive(Serialize, Deserialize)]
pub(super) struct TerminalControlPreview {
    pub(super) controls: [f32; 21],
    pub(super) raw_pd: [f32; 21],
    pub(super) raw_compensated: [f32; 21],
    pub(super) pd_saturated: [bool; 21],
    pub(super) compensated_saturated: [bool; 21],
    pub(super) compensation_active: bool,
}

impl TerminalControlPreview {
    #[cfg(feature = "getup-terminal-base-ready-experiment")]
    pub(super) fn finite_active_unclipped(&self) -> bool {
        self.compensation_active
            && self.raw_pd.iter().chain(&self.raw_compensated).chain(&self.controls)
                .all(|x| x.is_finite())
            && !self.pd_saturated.iter().chain(&self.compensated_saturated).any(|x| *x)
    }
}

#[cfg(feature = "getup-terminal-base-ready-experiment")]
pub(super) fn terminal_control_preview(
    s: &Simulation,
    reference: &StandingReference,
    q_target: &[f32; 21],
    q: &[f32; 21],
    dq: &[f32; 21],
) -> Result<TerminalControlPreview, String> {
    // Call the original helper for validation and the exact first clamp.
    let pd = super::slow_controls(&s.model, q_target, &[0.0; 21], q, dq, true)?;
    let mut raw_pd = [0.0; 21];
    for part in &s.model.segments {
        if let Some(h) = part.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
            let i = h.qpos_index;
            raw_pd[h.actuator_index] = (q_target[i] + 0.0) - q[i] - 0.2 * dq[i];
        }
    }
    let support = support_reactions_in_frame(s, reference.com, reference.forward, reference.lateral);
    let mut raw_compensated = pd;
    if let Some(support) = support {
        for (control, feedforward) in raw_compensated.iter_mut().zip(compensation(s, support, q)) {
            *control += feedforward;
        }
    }
    Ok(TerminalControlPreview {
        controls: raw_compensated.map(|x| x.clamp(-1.0, 1.0)),
        raw_pd,
        raw_compensated,
        pd_saturated: raw_pd.map(|x| !x.is_finite() || x.abs() >= 1.0),
        compensated_saturated: raw_compensated.map(|x| !x.is_finite() || x.abs() >= 1.0),
        compensation_active: support.is_some(),
    })
}

#[cfg(feature = "getup-terminal-balance-experiment")]
pub(super) fn recovered_reference(s: &Simulation) -> Option<StandingReference> {
    let mut forward = s.world.bodies[s.bodies[0]].rotation().normalize() * Vector::Z;
    forward.y = 0.0;
    let forward = forward.try_normalize()?;
    let reference = StandingReference {
        com: s.center_of_mass(),
        forward,
        lateral: Vector::Y.cross(forward),
    };
    if !reference.com.is_finite() {
        return None;
    }
    support_reactions_in_frame(s, reference.com, reference.forward, reference.lateral)?;
    Some(reference)
}

fn support_reactions(s: &Simulation) -> Option<SupportReactions> {
    support_reactions_in_frame(s, s.rest_com, s.heading * Vector::Z, s.heading * Vector::X)
}

/// Existing polygon projection plus its degenerate point/segment cases.
/// This selects a resultant location, never creates a rectangular sole.
pub(super) fn contact_cop_projection(points:&[[f32;2]],desired:[f32;2])->Option<[f32;2]> {
    if let Some(p)=stepping::project_support(points,desired,0.) {return Some(p);}
    let a=*points.first()?;let b=*points.iter().max_by(|x,y| {
        let d=|p:&&[f32;2]|(p[0]-a[0]).powi(2)+(p[1]-a[1]).powi(2);d(x).total_cmp(&d(y))
    })?;
    let d=[b[0]-a[0],b[1]-a[1]];let n=d[0]*d[0]+d[1]*d[1];
    if n<1e-12 {return Some(a);}
    if points.iter().any(|p|((p[0]-a[0])*d[1]-(p[1]-a[1])*d[0]).abs()>1e-8) {return None;}
    let mut lo=0f32;let mut hi=0f32;
    for p in points {let t=((p[0]-a[0])*d[0]+(p[1]-a[1])*d[1])/n;lo=lo.min(t);hi=hi.max(t);}
    let t=(((desired[0]-a[0])*d[0]+(desired[1]-a[1])*d[1])/n).clamp(lo,hi);
    Some([a[0]+t*d[0],a[1]+t*d[1]])
}

/// Fan barycentrics of the EXISTING convex contact hull. A persistent contact
/// anchors the fan; crossing a shared triangle edge has identical limiting
/// weights, so contact ordering does not pick a new load split arbitrarily.
/// Point/segment degeneracies are represented without manufacturing area.
fn contact_barycentrics(points:&[[f32;2]],desired:[f32;2],anchor:usize)->Option<Vec<f32>> {
    if points.is_empty() || anchor>=points.len() {return None;}
    let mut hull=stepping::support_hull(points.to_vec());
    if hull.is_empty() {hull=points.to_vec();hull.sort_by(|a,b|a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));hull.dedup();}
    let index=|p:[f32;2]|points.iter().position(|q|*q==p).unwrap();
    let cross=|a:[f64;2],b:[f64;2]|a[0]*b[1]-a[1]*b[0];
    let sub=|a:[f32;2],b:[f32;2]|[a[0] as f64-b[0] as f64,a[1] as f64-b[1] as f64];
    let finish=|indices:&[usize],values:&[f64]|->Option<Vec<f32>> {
        if values.iter().any(|w|!w.is_finite() || *w< -1e-10) {return None;}
        let mut weights=vec![0f32;points.len()];let sum=values.iter().map(|w|w.max(0.)).sum::<f64>();if sum<=0. {return None;}
        for (&i,&w) in indices.iter().zip(values) {weights[i]+=(w.max(0.)/sum) as f32;}
        let p=points.iter().zip(&weights).fold([0f64;2],|r,(p,w)|[r[0]+p[0] as f64* *w as f64,r[1]+p[1] as f64* *w as f64]);
        if (p[0]-desired[0] as f64).hypot(p[1]-desired[1] as f64)>1e-5 {return None;}Some(weights)
    };
    if hull.len()==1 {return finish(&[index(hull[0])],&[1.]);}
    if hull.len()==2 {
        let d=sub(hull[1],hull[0]);let v=sub(desired,hull[0]);let n=d[0]*d[0]+d[1]*d[1];if n<=0. {return None;}
        let t=(v[0]*d[0]+v[1]*d[1])/n;return finish(&[index(hull[0]),index(hull[1])],&[1.-t,t]);
    }
    for edge in 0..hull.len() {
        let b=hull[edge];let c=hull[(edge+1)%hull.len()];let a=points[anchor];
        let u=sub(b,a);let v=sub(c,a);let p=sub(desired,a);let det=cross(u,v);
        if det.abs()<1e-14 {continue;}
        let wb=cross(p,v)/det;let wc=cross(u,p)/det;
        if let Some(w)=finish(&[anchor,index(b),index(c)],&[1.-wb-wc,wb,wc]) {return Some(w);}
    }
    // A near-collinear f32 contact triangle can be thinner than a coordinate
    // ulp. Even its f32 centroid may lie outside its f64 triangle. Admit only
    // an edge representation within the input's rounding uncertainty, not a
    // projected COP at a mechanically meaningful distance outside support.
    let scale=points.iter().flatten().chain(desired.iter()).map(|v|v.abs()).fold(1f32,f32::max) as f64;
    let rounding=4.*f32::EPSILON as f64*scale;
    for edge in 0..hull.len() {
        let a=hull[edge];let b=hull[(edge+1)%hull.len()];let d=sub(b,a);let v=sub(desired,a);
        let n=d[0]*d[0]+d[1]*d[1];if n<=0. {continue;}
        let t=((v[0]*d[0]+v[1]*d[1])/n).clamp(0.,1.);
        if (v[0]-t*d[0]).hypot(v[1]-t*d[1])<=rounding {
            if let Some(w)=finish(&[index(a),index(b)],&[1.-t,t]) {return Some(w);}
        }
    }
    None
}

/// Reception-only C allocation. The original C operating domain and gains
/// are retained. Reactions are desired torques only, never injected forces.
#[cfg(feature="getup-native-locomotion-experiment")]
pub(super) fn reception_requested(s:&Simulation,reference:&StandingReference,target:&[f32;21],anchor:Option<(usize,[f32;3])>)
 ->Result<([f32;21],serde_json::Value),String> {
 let (q,dq)=super::joint_state(s)?;let mut raw_pd=[0.;21];
 for part in &s.model.segments {if let Some(h)=part.reference.as_ref().and_then(|r|r.hinge.as_ref()) {
  raw_pd[h.actuator_index]=target[h.qpos_index]-q[h.qpos_index]-0.2*dq[h.qpos_index];
 }}
 if raw_pd.iter().any(|v|!v.is_finite()) {return Err("Nonfinite Reception C feedback".into());}
 let pd=raw_pd.map(|v|v.clamp(-1.,1.));
 let fallback=|reason:String| Ok((pd,serde_json::json!({"compensationActive":false,"controlMode":"C posture-PD only; no absent-contact reaction","reason":reason,"rawPd":raw_pd,"postureTarget":target,"pointForces":[],"theoreticalReactionsRealized":false})));
 if support_reactions_in_frame(s,reference.com,reference.forward,reference.lateral).is_none() {
  return fallback("outside unchanged C bilateral/upright compensation domain".into());
 }
 let ids=["left_foot","right_foot"].map(|n|s.model.segments.iter().position(|p|p.id==n).unwrap());
 let contacts=super::standing::contacts_including_unloaded(s)?;
 let current:Vec<_>=contacts.iter().filter(|c|ids.contains(&c.body)&&c.signed_gap_m<=0.).collect();
 let points:Vec<_>=current.iter().map(|c|[c.point.x,c.point.z]).collect();
 let com=s.center_of_mass();let twists=super::current_twists(s)?;
 let velocity=s.bodies.iter().zip(twists).map(|(h,(v,_))|v*s.world.bodies[*h].mass()).sum::<Vector>()/s.total_mass();
 let floor=s.world.colliders[s.floor].compute_aabb().maxs.y;let height=com.y-floor;
 if !height.is_finite() || height<=0.05 {return fallback("invalid C COM height".into());}
 let desired=com+((com-reference.com)*4.+velocity*4.)*(height/9.81);
 let Some(cop)=contact_cop_projection(&points,[desired.x,desired.z]) else{return fallback("no current support polygon".into());};
 let local=|c:&&super::standing::FootContact|s.world.bodies[s.bodies[c.body]].position().inverse_transform_point(c.point);
 let index=anchor.and_then(|(body,p)|current.iter().enumerate().filter(|(_,c)|c.body==body).min_by(|(_,a),(_,b)|local(a).distance_squared(Vector::from_array(p)).total_cmp(&local(b).distance_squared(Vector::from_array(p)))).map(|(i,_)|i)).unwrap_or(0);
 let Some(weights)=contact_barycentrics(&points,cop,index) else{return fallback("current COP representation unavailable".into());};
 let mut support=[(ids[0],Vector::ZERO,Vector::ZERO),(ids[1],Vector::ZERO,Vector::ZERO)];let mut fractions=[0.;2];
 for (c,w) in current.iter().zip(&weights) {let side=ids.iter().position(|i|*i==c.body).unwrap();fractions[side]+=w;support[side].1+=c.point* *w;}
 for side in 0..2 {if fractions[side]>0. {support[side].1/=fractions[side];support[side].2=Vector::Y*(fractions[side]*s.total_mass()*9.81);}}
 let (dynamic,acceleration)=dynamic_reactions(s,support);
 let (force_residual,moment_residual)=residual(s,dynamic,acceleration);
 let total:f32=support.iter().map(|(_,_,f)|f.y).sum();
 let actual_cop=support.iter().map(|(_,p,f)|*p*(f.y/total)).sum::<Vector>();
 let forces:Vec<_>=weights.iter().map(|w|(Vector::Y*9.81+acceleration)*(s.total_mass()* *w)).collect();
 for (c,f) in current.iter().zip(&forces) {let n=c.normal.normalize();let normal=f.dot(n);
  if normal<0. || (*f-n*normal).length()>normal || s.world.colliders[s.floor].friction()!=1. || s.world.colliders[s.colliders[c.body]].friction()!=1. {return fallback("C requested reaction outside unchanged friction domain".into());}
 }
 let feed=compensation(s,support,&q);let raw=std::array::from_fn(|i|pd[i]+feed[i]);
 if raw.iter().any(|v|!v.is_finite()) {return Err("Nonfinite Reception total request".into());}
 let anchor_contact=current[index];let anchor_local=local(&anchor_contact);
 Ok((raw,serde_json::json!({"compensationActive":true,"controlMode":"C PD plus governed-current-contact compensation","rawPd":raw_pd,"compensationControls":feed,"postureTarget":target,"anchor":(anchor_contact.body,anchor_local.to_array()),"requestedCopXZ":[desired.x,desired.z],"commandedCopXZ":[actual_cop.x,actual_cop.z],"copResidualM":[desired.x-actual_cop.x,desired.z-actual_cop.z],"predictedComAcceleration":acceleration.to_array(),"fractions":fractions,"pointForces":current.iter().zip(&forces).map(|(c,f)|serde_json::json!({"body":s.model.segments[c.body].id,"point":c.point.to_array(),"force":f.to_array()})).collect::<Vec<_>>(),"forceResidualN":force_residual.to_array(),"momentResidualNm":moment_residual.to_array(),"groupedReactions":dynamic.map(|(i,p,f)|(i,p.to_array(),f.to_array())),"theoreticalReactionsRealized":false})))
}

fn support_reactions_in_frame(
    s: &Simulation,
    reference_com: Vector,
    forward: Vector,
    lateral: Vector,
) -> Option<SupportReactions> {
    if !s.world.colliders[s.floor].is_enabled() {
        return None;
    }
    let feet = ["left_foot", "right_foot"]
        .map(|name| s.model.segments.iter().position(|p| p.id == name).unwrap());
    let torso_up = (s.world.bodies[s.bodies[0]].rotation().normalize() * Vector::Y).y;
    if torso_up < 0.9
        || feet.iter().any(|i| s.integrated_support_loads_n[*i] < 3.0)
        || s.integrated_support_loads_n
            .iter()
            .enumerate()
            .any(|(i, load)| !feet.contains(&i) && *load >= 3.0)
    {
        return None;
    }
    // Aggregate support can include playground platforms. This controller uses
    // the flat floor plane, so both feet must actually load this floor collider.
    for foot in feet {
        if measurement::floor_normal_impulse(&s.world, s.floor, s.colliders[foot]).ok()?
            / s.profile.dt_s
            < 3.0
        {
            return None;
        }
    }
    let com = s.center_of_mass();
    let twists = super::current_twists(s).ok()?;
    let com_velocity = s
        .bodies
        .iter()
        .zip(&twists)
        .map(|(handle, (velocity, _))| *velocity * s.world.bodies[*handle].mass())
        .sum::<Vector>()
        / s.total_mass();
    // Ankle-strategy feedback on measured COM: LIPM desired acceleration
    // a_h = -4(c-c_ref)-4v (2 rad/s critically damped intention). The associated
    // COP is c-h*a_h/g, clamped inside the grounded physical sole segment.
    // This is a joint-torque intention, never an external root force.
    let height = com.y - s.world.colliders[s.floor].compute_aabb().maxs.y;
    let desired_cop =
        com + ((com - reference_com) * 4.0 + com_velocity * 4.0) * (height.max(0.0) / 9.81);
    let mut points = [Vector::ZERO; 2];
    let floor_bounds = s.world.colliders[s.floor].compute_aabb();
    for side in 0..2 {
        let body = &s.world.bodies[s.bodies[feet[side]]];
        let primitives = &s.model.segments[feet[side]].reference.as_ref()?.primitives;
        if primitives.is_empty() {
            return None;
        }
        // A fixed anatomical sole segment is the COP intention. It does not
        // claim the heel is currently loaded. Using the solver's changing set
        // of positive-impulse points here cut ankle torque during a feature swap.
        // Current bilateral resolved loads gate control; only independent solved
        // contacts, never these geometric points, qualify physical support.
        let bottom = |point, radius| {
            body.translation() + body.rotation().normalize() * vector(point) - Vector::Y * radius
        };
        let back = primitives
            .iter()
            .map(|p| bottom(p.a, p.radius))
            .sum::<Vector>()
            / primitives.len() as f32;
        let front = primitives
            .iter()
            .map(|p| bottom(p.b, p.radius))
            .sum::<Vector>()
            / primitives.len() as f32;
        let delta = front - back;
        let forward_span = delta.dot(forward);
        if forward_span <= 0.05 {
            return None;
        }
        // Clip the geometric segment to the finite floor footprint. The material
        // endpoint height is an intention, not a contact/ray or ground-force proof.
        let mut lower = 0.0_f32;
        let mut upper = 1.0_f32;
        for axis in [0, 2] {
            if delta[axis].abs() < 1.0e-8 {
                if back[axis] < floor_bounds.mins[axis] || back[axis] > floor_bounds.maxs[axis] {
                    return None;
                }
            } else {
                let a = (floor_bounds.mins[axis] - back[axis]) / delta[axis];
                let b = (floor_bounds.maxs[axis] - back[axis]) / delta[axis];
                lower = lower.max(a.min(b));
                upper = upper.min(a.max(b));
            }
        }
        if lower > upper {
            return None;
        }
        let fraction = ((desired_cop - back).dot(forward) / forward_span).clamp(lower, upper);
        points[side] = back + delta * fraction;
    }
    let span = (points[0] - points[1]).dot(lateral);
    if span.abs() < 0.01 {
        return None;
    }
    let left_fraction = ((desired_cop - points[1]).dot(lateral) / span).clamp(0.1, 0.9);
    Some([
        (
            feet[0],
            points[0],
            Vector::Y * (left_fraction * s.total_mass() * 9.81),
        ),
        (
            feet[1],
            points[1],
            Vector::Y * ((1.0 - left_fraction) * s.total_mass() * 9.81),
        ),
    ])
}

/// Complete the existing COP intention with its consistent centroidal
/// acceleration. These are desired wrenches used only in joint inverse dynamics.
/// No force is applied to any body by this calculation.
#[cfg(feature = "standing-dynamics-experiment")]
fn dynamic_reactions(s: &Simulation, mut support: SupportReactions) -> (SupportReactions, Vector) {
    let total_vertical: f32 = support.iter().map(|(_, _, f)| f.y).sum();
    let cop = support.iter().map(|(_, p, f)| *p * (f.y / total_vertical)).sum::<Vector>();
    let com = s.center_of_mass();
    // Intended material endpoints retain their actual heights. Use the height
    // above the actual clamped resultant COP, not an assumed floor-plane point.
    let height = com.y - cop.y;
    if height <= 0.05 || !height.is_finite() {
        return (support, Vector::ZERO);
    }
    let acceleration = Vector::new(com.x - cop.x, 0.0, com.z - cop.z) * (9.81 / height);
    if acceleration != Vector::ZERO {
        for (_, _, force) in &mut support {
            *force += acceleration * (force.y / 9.81);
        }
    }
    (support, acceleration)
}

fn compensation(s: &Simulation, support: SupportReactions, q: &[f32; 21]) -> [f32; 21] {
    #[cfg(feature = "standing-dynamics-experiment")]
    let (support, acceleration) = dynamic_reactions(s, support);
    let mut forces = [Vector::ZERO; 25];
    let mut moments = [Vector::ZERO; 25];
    for (i, handle) in s.bodies.iter().enumerate() {
        let body = &s.world.bodies[*handle];
        forces[i] = Vector::NEG_Y * (body.mass() * 9.81);
        #[cfg(feature = "standing-dynamics-experiment")]
        if acceleration != Vector::ZERO {
            // Matched D'Alembert force at each actual body COM. Adding desired
            // horizontal ground reaction alone would leave a free-root residual.
            forces[i] -= acceleration * body.mass();
        }
        moments[i] = (body.rotation().normalize() * body.local_center_of_mass()).cross(forces[i]);
    }
    for (i, point, force) in support {
        forces[i] += force;
        moments[i] += (point - s.world.bodies[s.bodies[i]].translation()).cross(force);
    }
    let mut result = [0.0; 21];
    // The canonical source tree is topologically ordered. Accumulate distal
    // wrenches toward the root rather than resumming each descendant subtree.
    for i in (1..s.bodies.len()).rev() {
        let part = &s.model.segments[i];
        let parent = s
            .model
            .segments
            .iter()
            .position(|p| Some(&p.id) == part.parent.as_ref())
            .unwrap();
        let body = &s.world.bodies[s.bodies[i]];
        let parent_body = &s.world.bodies[s.bodies[parent]];
        if let Some(hinge) = part.reference.as_ref().and_then(|r| r.hinge.as_ref()) {
            let joint = part.joint_profile.as_ref().unwrap();
            let parent_rotation = parent_body.rotation().normalize();
            let axis =
                (parent_rotation * rotation(joint.parent_frame.rotation) * Vector::X).normalize();
            let anchor =
                parent_body.translation() + parent_rotation * vector(joint.parent_frame.anchor);
            let external = axis.dot(moments[i] + (body.translation() - anchor).cross(forces[i]));
            let passive = joint.axes[0].passive_stiffness_nm_per_rad * q[hinge.qpos_index];
            result[hinge.actuator_index] = (passive - external) / hinge.gear;
        }
        let distal_force = forces[i];
        let distal_moment =
            moments[i] + (body.translation() - parent_body.translation()).cross(distal_force);
        forces[parent] += distal_force;
        moments[parent] += distal_moment;
    }
    result
}

#[cfg(feature = "standing-dynamics-experiment")]
fn residual(s: &Simulation, support: SupportReactions, acceleration: Vector) -> (Vector, Vector) {
    let com = s.center_of_mass();
    let mut force = Vector::ZERO;
    let mut moment = Vector::ZERO;
    for handle in &s.bodies {
        let b = &s.world.bodies[*handle];
        let f = Vector::NEG_Y * (b.mass() * 9.81) - acceleration * b.mass();
        let c = b.translation() + b.rotation().normalize() * b.local_center_of_mass();
        force += f;
        moment += (c - com).cross(f);
    }
    for (_, p, f) in support {
        force += f;
        moment += (p - com).cross(f);
    }
    (force, moment)
}

#[cfg(feature = "standing-dynamics-experiment")]
pub fn dynamic_balance_diagnostic(s: &Simulation) -> serde_json::Value {
    let Some(static_support) = support_reactions(s) else { return serde_json::json!({"active":false,"physicsIntegrations":0}); };
    let (support, acceleration) = dynamic_reactions(s, static_support);
    let (force, moment) = residual(s, support, acceleration);
    let vertical: f32 = static_support.iter().map(|(_, _, f)| f.y).sum();
    let cop = static_support.iter().map(|(_, p, f)| *p * (f.y / vertical)).sum::<Vector>();
    serde_json::json!({"active":true,"physicsIntegrations":0,"accelerationMps2":vec3(acceleration),"resultantCop":vec3(cop),"loadFractions":static_support.map(|(_,_,f)|f.y/vertical),"intendedForceResidualN":vec3(force),"intendedCentroidalMomentResidualNm":vec3(moment),"intendedFrictionRatio":acceleration.length()/9.81,"supportIntentions":support.map(|(i,p,f)|serde_json::json!({"body":s.model.segments[i].id,"point":vec3(p),"forceN":vec3(f)}))})
}

/// Algebraic counterfactual fixtures only, never integrated or used as support
/// evidence. Exercise unequal loads, endpoint heights, and clamped-resultant COPs.
#[cfg(feature = "standing-dynamics-experiment")]
pub fn dynamic_balance_fixture_invariants() -> Result<serde_json::Value, String> {
    let mut results = vec![];
    for heading in [0.0, std::f32::consts::PI/3.0, -std::f32::consts::PI/4.0] {
        let s = Simulation::new_protocol(heading, Profile::default(), true)?;
        let before = serde_json::to_value(s.snapshot()).unwrap();
        let feet = ["left_foot", "right_foot"].map(|name|s.model.segments.iter().position(|p|p.id==name).unwrap());
        let com = s.center_of_mass();
        for (x,z,fraction) in [(0.0,0.0,0.5),(0.1,0.0,0.1),(-0.1,0.0,0.9),(0.0,0.1,0.1),(0.0,-0.1,0.9)] {
            let offset = s.heading * Vector::new(x,0.0,z);
            let cop = Vector::new(com.x+offset.x,0.02,com.z+offset.z);
            let lateral = s.heading * Vector::X * 0.1;
            let p0 = cop + lateral * (1.0-fraction) + Vector::Y*0.01;
            let p1 = cop - lateral * fraction - Vector::Y*(0.01*fraction/(1.0-fraction));
            let weight = s.total_mass()*9.81;
            let static_support = [(feet[0],p0,Vector::Y*(fraction*weight)),(feet[1],p1,Vector::Y*((1.0-fraction)*weight))];
            let (support, acceleration) = dynamic_reactions(&s, static_support);
            let (force,moment) = residual(&s,support,acceleration);
            if force.length()>0.001 || moment.length()>0.001 {
                return Err(format!("Intended wrench did not close: force={force:?}, moment={moment:?}"));
            }
            if x==0.0 && z==0.0 && acceleration.length()>1e-6 { return Err("Zero-acceleration statics changed".into()); }
            results.push(serde_json::json!({"heading":heading,"x":x,"z":z,"leftFraction":fraction,"accelerationMps2":vec3(acceleration),"forceResidualN":vec3(force),"momentResidualNm":vec3(moment)}));
        }
        if before!=serde_json::to_value(s.snapshot()).unwrap() || s.substeps_completed!=0 {return Err("Algebraic fixture modified the simulation".into());}
    }
    Ok(serde_json::json!({"physicsIntegrations":0,"passed":true,"fixtures":results}))
}

pub(crate) fn controls(
    s: &Simulation,
    reference: &[f32; 21],
    q: &[f32; 21],
    dq: &[f32; 21],
) -> Result<[f32; 21], String> {
    let mut controls = super::slow_controls(&s.model, reference, &[0.0; 21], q, dq, true)?;
    if let Some(support) = support_reactions(s) {
        for (control, feedforward) in controls.iter_mut().zip(compensation(s, support, q)) {
            *control = (*control + feedforward).clamp(-1.0, 1.0);
        }
    }
    Ok(controls)
}

#[cfg(feature = "getup-terminal-balance-experiment")]
pub(super) fn controls_with_reference(
    s: &Simulation,
    reference: &StandingReference,
    q_target: &[f32; 21],
    q: &[f32; 21],
    dq: &[f32; 21],
) -> Result<([f32; 21], bool), String> {
    let mut controls = super::slow_controls(&s.model, q_target, &[0.0; 21], q, dq, true)?;
    let support = support_reactions_in_frame(s, reference.com, reference.forward, reference.lateral);
    if let Some(support) = support {
        for (control, feedforward) in controls.iter_mut().zip(compensation(s, support, q)) {
            *control = (*control + feedforward).clamp(-1.0, 1.0);
        }
    }
    Ok((controls, support.is_some()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nominal(s: &Simulation) -> ([f32; 21], [f32; 21], [f32; 21]) {
        let reference = super::super::controller::standing_target(&s.model)[..21]
            .try_into()
            .unwrap();
        let (q, dq) = super::super::joint_state(s).unwrap();
        (reference, q, dq)
    }

    #[test]
    fn unsupported_floor_loss_fallen_and_nonfoot_load_use_the_exact_source_pd() {
        let mut s = Simulation::new(0.0, Profile::default(), true).unwrap();
        let original = |s: &Simulation| {
            let (reference, q, dq) = nominal(s);
            super::super::slow_controls(&s.model, &reference, &[0.0; 21], &q, &dq, true).unwrap()
        };
        let actual = |s: &Simulation| {
            let (reference, q, dq) = nominal(s);
            controls(s, &reference, &q, &dq).unwrap()
        };
        assert_eq!(actual(&s), original(&s));
        for _ in 0..8 {
            s.advance_tick().unwrap();
        }
        assert!(support_reactions(&s).is_some());
        let before = serde_json::to_value(s.snapshot()).unwrap();
        assert!(actual(&s).iter().all(|v| v.is_finite() && v.abs() <= 1.0));
        assert_eq!(
            serde_json::to_value(s.snapshot()).unwrap(),
            before,
            "read-only controller"
        );
        s.world.colliders[s.floor].set_enabled(false);
        assert_eq!(actual(&s), original(&s));
        s.world.colliders[s.floor].set_enabled(true);
        s.integrated_support_loads_n[0] = 3.0;
        assert_eq!(actual(&s), original(&s));
        s.integrated_support_loads_n[0] = 0.0;
        // Negative read-only fixture only; never integrated as a continuous trial.
        s.world.bodies[s.bodies[0]]
            .set_rotation(Rotation::from_rotation_x(std::f32::consts::PI), false);
        assert_eq!(actual(&s), original(&s));
    }

    #[test]
    fn com_error_and_generalized_velocity_independently_restore_ankle_torque() {
        let mut s = Simulation::new(0.0, Profile::default(), true).unwrap();
        for _ in 0..8 {
            s.advance_tick().unwrap();
        }
        let id = *s
            .world
            .multibody_joints
            .rigid_body_link(s.bodies[0])
            .unwrap();
        s.world
            .multibody_joints
            .get_multibody_mut(id.multibody)
            .unwrap()
            .generalized_velocity_mut()
            .fill(0.0);
        s.rest_com = s.center_of_mass();
        let (reference, q, dq) = nominal(&s);
        let neutral = controls(&s, &reference, &q, &dq).unwrap();
        let original_reference = s.rest_com;
        s.rest_com.z -= 0.01;
        let positive_error = controls(&s, &reference, &q, &dq).unwrap();
        s.rest_com = original_reference;
        // Counterfactual velocity fixture only, no integration or body-cache
        // synchronization. A uniform +Z root speed must be observed directly.
        s.world
            .multibody_joints
            .get_multibody_mut(id.multibody)
            .unwrap()
            .generalized_velocity_mut()[2] = 0.05;
        let positive_velocity = controls(&s, &reference, &q, &dq).unwrap();
        for qpos in [7, 13] {
            let actuator = s
                .model
                .segments
                .iter()
                .filter_map(|p| p.reference.as_ref()?.hinge.as_ref())
                .find(|h| h.qpos_index == qpos)
                .unwrap()
                .actuator_index;
            assert!(positive_error[actuator] > neutral[actuator] + 0.01);
            assert!(positive_velocity[actuator] > neutral[actuator] + 0.01);
        }
    }

    #[test]
    fn recursive_compensation_matches_independent_descendant_wrench_sum() {
        let mut s = Simulation::new(0.0, Profile::default(), true).unwrap();
        for _ in 0..8 {
            s.advance_tick().unwrap();
        }
        let support = support_reactions(&s).unwrap();
        let (_, q, _) = nominal(&s);
        let recursive = compensation(&s, support, &q);
        #[cfg(feature = "standing-dynamics-experiment")]
        let (support, acceleration) = dynamic_reactions(&s, support);
        let mut maximum_error = 0.0_f32;
        for (i, part) in s.model.segments.iter().enumerate() {
            let Some(h) = part.reference.as_ref().and_then(|r| r.hinge.as_ref()) else {
                continue;
            };
            let parent = s
                .model
                .segments
                .iter()
                .position(|p| Some(&p.id) == part.parent.as_ref())
                .unwrap();
            let joint = part.joint_profile.as_ref().unwrap();
            let parent_body = &s.world.bodies[s.bodies[parent]];
            let axis = (parent_body.rotation().normalize()
                * rotation(joint.parent_frame.rotation)
                * Vector::X)
                .normalize();
            let anchor = parent_body.translation()
                + parent_body.rotation().normalize() * vector(joint.parent_frame.anchor);
            let descendant = |mut body: usize| {
                loop {
                    if body == i {
                        return true;
                    }
                    let Some(parent) = &s.model.segments[body].parent else {
                        return false;
                    };
                    body = s
                        .model
                        .segments
                        .iter()
                        .position(|p| &p.id == parent)
                        .unwrap();
                }
            };
            let mut external = 0.0;
            for (j, handle) in s.bodies.iter().enumerate() {
                if descendant(j) {
                    let body = &s.world.bodies[*handle];
                    let center = body.translation()
                        + body.rotation().normalize() * body.local_center_of_mass();
                    let force = Vector::NEG_Y * body.mass() * 9.81;
                    #[cfg(feature = "standing-dynamics-experiment")]
                    let force = force - acceleration * body.mass();
                    external += axis.dot((center - anchor).cross(force));
                }
            }
            for (foot, point, force) in support {
                if descendant(foot) {
                    external += axis.dot((point - anchor).cross(force));
                }
            }
            let expected =
                (joint.axes[0].passive_stiffness_nm_per_rad * q[h.qpos_index] - external) / h.gear;
            maximum_error = maximum_error.max((recursive[h.actuator_index] - expected).abs());
        }
        assert!(
            maximum_error < 0.000002,
            "normalized torque mismatch {maximum_error}"
        );
    }

    #[cfg(feature = "standing-dynamics-experiment")]
    #[test]
    fn dynamic_intention_closes_wrench_and_preserves_zero_acceleration_statics() {
        let report = dynamic_balance_fixture_invariants().unwrap();
        assert_eq!(report["passed"], true);
        assert_eq!(report["physicsIntegrations"], 0);
        assert_eq!(report["fixtures"].as_array().unwrap().len(), 15);
        let s = Simulation::new_protocol(0.0, Profile::default(), true).unwrap();
        let com = s.center_of_mass();
        let point = Vector::new(com.x, 0.0, com.z);
        let force = Vector::Y * (s.total_mass() * 9.81 * 0.5);
        let support = [(10, point, force), (16, point, force)];
        let (actual, acceleration) = dynamic_reactions(&s, support);
        assert_eq!(acceleration, Vector::ZERO);
        assert_eq!(actual, support, "zero acceleration must leave every static wrench word unchanged");
    }

    #[test]
    fn default_contact_standing_survives_three_headings_without_external_assistance() {
        for heading in [0.0, std::f32::consts::PI / 3.0, -std::f32::consts::PI / 4.0] {
            let mut s = Simulation::new_protocol(heading, Profile::default(), true).unwrap();
            let mut anchors = None;
            let mut loaded_slip = [0.0_f64; 2];
            let mut observed = 0;
            for _ in 0..200 {
                s.advance_tick_observed(|state, substep| {
                    let diagnostic =
                        super::super::controller::standing_diagnostic(state, anchors).unwrap();
                    // Cumulative loaded slip starts at the first integration,
                    // never at the settled anchor and never at each sample.
                    for (side, integral) in loaded_slip.iter_mut().enumerate() {
                        let foot = &diagnostic["contactMotion"]["feet"][side];
                        if foot["loaded"] == true {
                            if let Some(speed) = foot["impulseWeightedTangentSpeedMps"].as_f64() {
                                *integral += speed * f64::from(state.profile.dt_s);
                            }
                        }
                        assert!(
                            *integral <= 0.02,
                            "heading {heading}, side {side}: loaded slip {integral} m"
                        );
                    }
                    if state.substeps_completed >= 400 {
                        if anchors.is_none() {
                            anchors =
                                Some(serde_json::from_value(diagnostic["feet"].clone()).unwrap());
                        }
                        measurement::standing_support(state, substep)?;
                        for (name, value) in diagnostic["predicates"].as_object().unwrap() {
                            if name == "footDriftFromSuppliedAnchors" && value.is_null() {
                                continue;
                            }
                            assert_eq!(
                                value, true,
                                "heading {heading} at {}: {name}",
                                state.substeps_completed
                            );
                        }
                        observed += 1;
                    }
                    Ok(())
                })
                .unwrap();
            }
            assert_eq!(s.substeps_completed, 4000);
            assert_eq!(observed, 3601);
            assert!(s.getup_controller.is_none());
            assert!(s.striker.as_ref().unwrap().snapshot(&s.world).available);
            println!(
                "Default standing heading {heading}: 4000 real Rapier integrations; final torso up {}, cumulative loaded slip {loaded_slip:?} m",
                super::super::physical_observation(&s).unwrap()[36]
            );
        }
    }
}

/// Separate transfer-only support intention. The bilateral standing functions
/// above remain unchanged. Measured contacts admit unloading; these hypothetical
/// reactions only contribute equal/opposite joint torques via compensation().
pub(super) fn transfer_controls(
    s: &Simulation, reference: &StandingReference, target: &[f32;21], retained_side:usize, transfer_fraction:f32,
)->Result<[f32;21],String>{
    if !cfg!(feature="getup-supported-transfer-experiment") || retained_side>1 || s.grab.is_some() {
        return Err("Unsupported or externally loaded gravity-only transfer".into());
    }
    let ids=["left_foot","right_foot"].map(|name|s.model.segments.iter().position(|p|p.id==name).unwrap());
    let contacts=super::standing::contacts(s)?;
    let actual=ids.map(|i|contacts.iter().filter(|c|c.body==i).map(|c|c.impulse).sum::<f32>()/s.profile.dt_s);
    if actual[retained_side]<3. || s.integrated_support_loads_n.iter().enumerate().any(|(i,l)|!ids.contains(&i)&&*l>=3.) {
        return Err("Transfer has no actual retained-foot-only floor support".into());
    }
    let com=s.center_of_mass();let twists=super::current_twists(s)?;
    let velocity=s.bodies.iter().zip(twists).map(|(h,(v,_))|v*s.world.bodies[*h].mass()).sum::<Vector>()/s.total_mass();
    let height=com.y-s.world.colliders[s.floor].compute_aabb().maxs.y;
    if height<=0.05 || !height.is_finite(){return Err("Invalid transfer COM height".into());}
    let desired=com+((com-reference.com)*4.+velocity*4.)*(height/9.81);
    let mut points=[Vector::ZERO;2];
    for side in 0..2 {
        let b=&s.world.bodies[s.bodies[ids[side]]];
        let primitives=&s.model.segments[ids[side]].reference.as_ref().ok_or("Missing sole geometry")?.primitives;
        if primitives.is_empty(){return Err("Missing sole primitives".into());}
        let point=|p,r|b.translation()+b.rotation().normalize()*vector(p)-Vector::Y*r;
        let back=primitives.iter().map(|p|point(p.a,p.radius)).sum::<Vector>()/primitives.len() as f32;
        let front=primitives.iter().map(|p|point(p.b,p.radius)).sum::<Vector>()/primitives.len() as f32;
        let delta=front-back;let span=delta.dot(reference.forward);
        if span<=0.05 {return Err("Invalid transfer sole orientation".into());}
        points[side]=back+delta*((desired-back).dot(reference.forward)/span).clamp(0.,1.);
        let floor=s.world.colliders[s.floor].compute_aabb();
        if points[side].x<floor.mins.x || points[side].x>floor.maxs.x || points[side].z<floor.mins.z || points[side].z>floor.maxs.z {
            return Err("Transfer intention outside finite floor".into());
        }
    }
    let span=(points[0]-points[1]).dot(reference.lateral);
    if span.abs()<0.01{return Err("Degenerate transfer sole span".into());}
    let minimum_fraction=0.1*(1.-transfer_fraction);
    let fraction=if actual[1-retained_side]<3. {if retained_side==0 {1.}else{0.}}
        else {((desired-points[1]).dot(reference.lateral)/span).clamp(minimum_fraction,1.-minimum_fraction)};
    let weight=s.total_mass()*9.81;
    let support=[(ids[0],points[0],Vector::Y*(weight*fraction)),(ids[1],points[1],Vector::Y*(weight*(1.-fraction)))];
    let (q,dq)=super::joint_state(s)?;let feed=compensation(s,support,&q);
    let mut controls=[0.;21];
    for p in &s.model.segments {if let Some(h)=p.reference.as_ref().and_then(|r|r.hinge.as_ref()){
        let pd=target[h.qpos_index]-q[h.qpos_index]-0.2*dq[h.qpos_index];
        let raw=pd+feed[h.actuator_index];
        if !pd.is_finite() || pd.abs()>=1. || !raw.is_finite() || raw.abs()>0.95 {
            return Err("Transfer command lacks finite unclipped 5% normalized torque reserve".into());
        }
        controls[h.actuator_index]=raw;
    }}
    Ok(controls)
}

#[cfg(all(test,feature="getup-supported-transfer-experiment"))]
mod transfer_wrench_tests {
    use super::*;
    #[test]
    fn either_single_support_intention_closes_root_wrench_without_body_force(){
        let s=Simulation::new_protocol(0.4,Profile::default(),true).unwrap();
        let before=bincode::serialize(&s.world).unwrap();let com=s.center_of_mass();
        let feet=["left_foot","right_foot"].map(|name|s.model.segments.iter().position(|p|p.id==name).unwrap());
        for fraction in [0.,0.1,0.5,0.9,1.] {
            let support=[(feet[0],Vector::new(com.x-0.08,0.,com.z),Vector::Y*(s.total_mass()*9.81*fraction)),
                (feet[1],Vector::new(com.x+0.08,0.,com.z),Vector::Y*(s.total_mass()*9.81*(1.-fraction)))];
            let (support,a)=dynamic_reactions(&s,support);let (f,m)=residual(&s,support,a);
            assert!(f.length()<0.001 && m.length()<0.001);
            let (q,_)=super::super::joint_state(&s).unwrap();
            assert!(compensation(&s,[(feet[0],Vector::new(com.x-0.08,0.,com.z),Vector::Y*(s.total_mass()*9.81*fraction)),
                (feet[1],Vector::new(com.x+0.08,0.,com.z),Vector::Y*(s.total_mass()*9.81*(1.-fraction)))],&q).iter().all(|x|x.is_finite()));
        }
        assert_eq!(before,bincode::serialize(&s.world).unwrap());assert_eq!(s.substeps_completed,0);
    }
}
