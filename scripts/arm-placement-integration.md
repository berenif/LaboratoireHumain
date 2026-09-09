# Minimal arm-placement integration

The reusable geometry is in `src/character/recovery-support.ts`: `usableRecoveryArmSupport`, `reachableArmBraceTarget`, and `RecoveryArmBraceTarget`. The placement below reached two loaded hands at 1.87 s in the prone-left fixture, with 6–11 mm foot drift and zero upward assistance during roll. The later press remains unsuccessful; do not copy the experimental controller's press or gain tuning.

Add these fields:

```ts
private placingProneArms = false;
private placementRootRotation: Quat = { x: 0, y: 0, z: 0, w: 1 };
private placementTorsoRotation: Quat = { x: 0, y: 0, z: 0, w: 1 };
private armBraces = new Map<Side, RecoveryArmBraceTarget>();
```

At `chooseRoute` after selection, capture once:

```ts
this.armBraces.clear();
for (const side of SIDES) {
  const target = reachableArmBraceTarget(side, poses, this.heading);
  if (target?.floorReachable && target.jointLimitErrorRad < 1e-6)
    this.armBraces.set(side, target);
}
this.placementRootRotation = { ...pelvis.rotation };
this.placementTorsoRotation = quatMultiply(quatInverse(pelvis.rotation), poses.get("torso")!.rotation);
this.placingProneArms = selected.route === "prone" && !SIDES.every(side =>
  usableRecoveryArmSupport(side, poses, this.data.contacts.filter(c => c.segment.endsWith("Hand"))));
```

When `placingProneArms`, enter existing `roll` / `roll` rather than the prone brace shortcut. Do not override crouch or half-kneel route priority. A supine/side route can call the same preparation after physically rolling prone.

Add measured completion helper:

```ts
private armPlacementReady(side: Side, poses: Map<SegmentId, SegmentPose>): boolean {
  const target = this.armBraces.get(side);
  return !!target && usableRecoveryArmSupport(side, poses, this.data.contacts.filter(c =>
    c.segment.endsWith("Hand") && !this.released.has(c.segment)))
    && length(sub(poses.get(`${side}Hand`)!.position, target.position)) < .14;
}
```

At the start of `transfer`, while roll/preparing:

```ts
this.rootGoal = { ...pelvis.position }; // no upward rolling root target
this.rootRotation = this.placementRootRotation;
for (const side of [this.data.rollSide!, this.data.rollSide === "left" ? "right" : "left"] as Side[]) {
  if (this.armPlacementReady(side, poses)) continue;
  const target = this.armBraces.get(side);
  if (!target) continue;
  // Call release before the already-released test: a forearm can recontact
  // while the hand is moving and must not become an accidental frozen anchor.
  if (this.release([`${side}Hand`, `${side}Forearm`], poses)
      || (this.released.has(`${side}Hand`) && this.released.has(`${side}Forearm`))) {
    this.placements.set(`${side}Hand`, {
      ...target.position, y: target.position.y + (this.stageTime < 1 ? .15 : 0),
    });
    this.bend.set(`${side}UpperArm`, rotate(quatInverse(poses.get("torso")!.rotation), target.bend));
  }
}
return;
```

The contact-clearance raise is a hand target, not pelvis assistance. The experiment's one-second switch is only a starting point; prefer a smooth clearance path followed by measured contact establishment. Final floor targets remain captured; never recalculate them from a sliding hand.

While preparing, joint actuation must keep the measured entry torso and all thigh/shin/foot relative targets. `limbTargets` must use `placementTorsoRotation` instead of the ordinary rolling torso pitch, and use `armBraces.get(side)?.rotation` for the unplanted hand endpoint. Preserve the existing `plant.rotation` override once contact is established.

Do not allocate full body weight to feet or hands while pelvis/torso/thighs still rest on the floor. The successful placement run had feedforward disabled during this preparation; the ordinary bounded feedback still moved the arms. The new bounded whole-body load solver may replace this special case if it correctly allocates floor reactions to resting body contacts.

Allow the roll→brace shortcut only when both `armPlacementReady` checks pass, then deliberately release forearms with existing projected remaining-support geometry while still in roll. If loaded forearms cannot be safely released, retain the stage. Enter brace, clear `placingProneArms`, and blend from the measured entry joints/root as usual. The broader existing roll→brace physical conditions remain for non-placement routes.

Finally, include hand placement distance in stalled-progress measurement while preparing; pelvis height/updot need not improve during successful hand placement.
