# Lean publication, 17 October 2026

This publication consolidates the final code from local source commit `d81d0fcbe897e1ae7f0a83b0e50b18a512143fd8` onto the previously published `c443801cba1b55cb4b3c36a3f7590feb1e2ac109`. Experimental controller scripts and production Rust remain as in that source, apart from the targeted terrain-spawn construction correction described below. A subsequent CI-only smoke-contract fix derives the 16 rendered meshes and 25 pose slots from the canonical GetUp model, retains all pose identities, enforces the existing five-second limit before advancement and reports native substeps from the native clock. Seven focused regression tests cover those contracts; application physics and thresholds are unchanged. A separate initialization-only correction preserves the selected terrain elevation during GetUp posture normalization; its regression covers all 21 station/difficulty combinations without integration. The original investigation branch, commits and raw files are preserved locally.

Concise text results, experiment configurations, ledgers, checksum manifests and required pure-test fixtures are included. Verbose intermediate diagnostics remain in the original local history alongside raw files. The additional 168,306,426 bytes of compressed raw traces, checkpoints, video and images are intentionally excluded from this branch to keep the repository compact. Their paths, sizes, Git blob IDs and SHA-256 checksums are recorded in `publication-2026-10-17-raw-manifest.json`; that manifest is an inventory, not a remote download source. Reproducing physical runs requires the retained local inputs and fresh explicit authorization. Existing Library comparison videos remain separate user artifacts.

Results remain experimental: the 3 cm measured transfer and five-second retained balance passed their stated test; the partial step did not validate landing or final standing, and the learned pilot was rejected. Default runtime controller selection and frozen GetUp weights are unchanged. Merging does not deploy Pages; promotion still needs the separate accepted-artifact workflow. The existing automatic application CI, including its bounded browser smoke test, remains enabled.

## Consolidated local commits

- `cbabe9cca858ca0712864b6406e320cd4892166c` Verify five-second GetUp standing hold and measure contact motion
- `5d66673fabc2981069752f44b09dccf5cce8384e` Record physical gate against premature GetUp settling
- `acfdaa220bb96cd742b0e1ff0cfb1f24751c07be` Add native GetUp-style stationary balance experiment
- `4af51de6381c64b5c5eb59dea8b4264d36c86d30` Run bounded GetUp SAC pilot and reject degraded balance candidate
- `ffbee8b2294a82b0a36cd9373f9c45573940102d` Preflight contact-gated GetUp recovery step without physics
- `e49b95f94266dd49d89349ab9fec750781f97b2a` Keep swing blend continuous across reference phases
- `aa49361510e3021596be4c51597e0f2a3915e90c` Harden recovery-step gates and paired evidence before rollout
- `774e6d2ab4f4cb948afe381c657db3ad921df09d` Record bounded recovery probes and validated equilibrium prerequisite
- `bde2d21f8b0cc65e937370f2a7659d0cb0950262` Close recovery session with guarded retreat and honest failed-step evidence
- `d81d0fcbe897e1ae7f0a83b0e50b18a512143fd8` Add measured-contact balance and retain bounded partial-step evidence
