/** The planned foot exists during transfer, before support.swingFoot does. */
export function stepLoadDiagnostics(snapshot, definitions, persistenceS = 0.05) {
  const step = snapshot.diagnostics.standingChain?.step;
  const movingFoot = step?.foot ?? snapshot.support.swingFoot;
  const movingSide = definitions.get(movingFoot)?.side ?? null;
  const contacts = snapshot.diagnostics.contactDiagnostics?.contacts
    ?? snapshot.diagnostics.recovery?.contacts ?? [];
  const soles = contacts.filter(contact => {
    const definition = definitions.get(contact.segment);
    return (definition?.role === "hindfoot" || definition?.role === "forefoot")
      && contact.normalY >= 0.65 && contact.forceN > 0;
  });
  const sum = (moving, qualified) => movingSide === null ? 0 : soles
    .filter(contact => (definitions.get(contact.segment).side === movingSide) === moving
      && (!qualified || (contact.loadBearing && contact.persistenceS >= persistenceS)))
    .reduce((total, contact) => total + contact.forceN, 0);
  return { movingFoot, movingSide, movingLoadN: sum(true, false),
    movingQualifiedLoadN: sum(true, true), retainedLoadN: sum(false, true),
    retainedNormalLoadN: sum(false, false) };
}
