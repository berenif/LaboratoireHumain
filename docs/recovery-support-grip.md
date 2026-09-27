# Recovery support grip

Hands, forearms (including twist segments), shins and ankles now use friction
1.2 instead of 0.45. Their Min combine rule previously limited floor contact to
0.45 despite the floor's coefficient of 4. The new value matches the recovery
load planner's friction budget on the flat floor. Soles remain at 4 and other
body segments at 0.45; lower-friction terrain still limits grip through Min.

The unchanged real-strike replay in
`evidence/recovery-grip-20260927-user-01/results.json` compares both materials
over 600 physics steps. Load-weighted accumulated support slip decreases from
0.5362 m to 0.4174 m (22.2%); accumulated torso horizontal travel decreases from
1.9972 m to 1.2419 m (37.8%). Peak segment speed changes from 7.973 to 8.121 m/s,
and peak post-landing pelvis height from 0.1222 to 0.1254 m.

Both trials reach floor support at tick 54 and finish in the roll phase with
zero completed recoveries. This is a grip improvement, not a completed get-up
repair or browser visual acceptance. The probe's contract retains source
fingerprints from before the material change.
