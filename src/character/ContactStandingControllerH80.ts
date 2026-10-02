import { ContactStandingController, CONTACT_STANDING } from "./ContactStandingController";
import type { ContactSolveInput } from "./coordinated-contact-solve";
import { solveContactForcesH80 } from "./coordinated-contact-solve-h80";

export const CONTACT_STANDING_H80 = Object.freeze({ ...CONTACT_STANDING, id: "h80-v1" });
/** Final bounded candidate: reuse geometry columns and optimize unchanged math. */
export class ContactStandingControllerH80 extends ContactStandingController {
  protected override get candidateId(): string { return CONTACT_STANDING_H80.id; }
  protected override get reuseContactColumns(): boolean { return true; }
  protected override allocate(input: ContactSolveInput) { return solveContactForcesH80(input); }
}
