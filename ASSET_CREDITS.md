# Software and source credits

The maintained runtime uses Leptos, wgpu and Rapier in Rust/WASM. Exact dependency versions are recorded in `rust/Cargo.lock`; vendored Rapier notices are retained under `rust/vendor/rapier3d/`. Browser test tooling is pinned in `package-lock.json`.

The humanoid and environment geometry are procedural. Experimental recovery references and their source attribution are preserved in the dated reports under `docs/` and the provenance records alongside the Rust data. They do not establish physical qualification.

Former React, Three.js, Vinext and JavaScript Rapier dependencies are available with the retired application in Git history.
