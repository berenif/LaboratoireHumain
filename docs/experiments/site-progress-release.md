# Private-site progress release (25 October 2026)

The playable runtime uses the byte-identical app, engine, GetUp cycle,
continuation and SLOW code previously delivered at 5dc93fa. The new visible
notice reports only the observed knee recovery, unqualified transfer/step,
and offline speed limitation. No walking controller is activated.

All experimental sources and their history remain recoverable at
4f02fe27ae6c093191354912b6e104932b1ae3ed (and the preserved local branch
experiment/pull-step-001). These offline probes depend on local evidence;
they are not runtime assets or claimed reproducible experiments from the
public site. Use that source revision when inspecting their original wiring.
The pure evaluator-contract repairs remain in the source history.

The release build excludes pull-step.js, scripts, local proofs, videos and
checkpoints except the two explicitly approved GetUp JSON runtime inputs.
An existing generated pull-step.js is preserved outside web/dist before build;
no unique evidence is deleted. Main runtime assets are same-origin.

Publisher handoff: use the exact public Git commit, without cloning a second
large checkout. In the retained checkout run npm run build, then
node scripts/verify-mujoco-package.mjs. Publish ONLY web/dist through the
official existing Sites helper. Preserve project_id
appgprj_6acabc12a7b08191b98888edf65d00f6 and owner-private audience.
Do not put repo scripts/proofs/node_modules into the static directory.
No deployment or media upload is performed by this release task.

Local checks: static package and policy fixtures; horizon algebra tests;
Python callback-only checks; portrait 360/390, landscape 844x390 and desktop
1366x900 in installed Chromium. The browser stays paused at zero integrated
steps while camera, details, targets, overflow and labelled report fixture are
checked. Screenshots were visually inspected. This is mobile emulation,
not an Android-device test or a newly executed recovery/step test.
Local audit and build hashes: web/proofs/mobile-progress-main/ (not published).

Disk accounting retains the fixed parent allowance until reconciled by its
owner. Cloud formula: du -sx -B1 /workspace minus baseline 419647488.
Conservative aggregate adds 13123543 allocated parent bytes. This cloud
cannot verify removal of those bytes on a separate publisher environment.
Opening/packaging peaks must fit the aggregate cap of 1530000000 before
using the helper; a source commit being ready does not establish that space.
