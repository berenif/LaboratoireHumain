import { ContactStandingController, CONTACT_STANDING } from "./ContactStandingController";
import type { ContactSolveInput } from "./coordinated-contact-solve";
import { solveContactForcesH79 } from "./coordinated-contact-solve-h79";

export const CONTACT_STANDING_H79 = Object.freeze({ ...CONTACT_STANDING, id: "h79-v1" });
/** Same H78 quadratic/constraints; certify the direct solution when admissible. */
export class ContactStandingControllerH79 extends ContactStandingController {
  protected override get candidateId(): string { return CONTACT_STANDING_H79.id; }
  protected override allocate(input: ContactSolveInput) { return solveContactForcesH79(input); }
}
