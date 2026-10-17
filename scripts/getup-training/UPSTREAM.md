# Pinned GetUp SAC source

`SAC.py` is retained byte-for-byte from [tianxintao/get_up_control](https://github.com/tianxintao/get_up_control/blob/5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e/SAC.py), commit `5ad52fbe1b5b7c56522a1aa10ccf17fb81c0d79e`.

SHA-256: `64b1abe69edaac1e3c3f9064f11bcb3d1168eb1d829117dd415eaa204dee0420`.

The adapter retains the upstream actor, twin critic and SAC update. It supplies a bounded native environment, fresh critics, capped exploration, critic warmup and finite-gradient checks. It does not use upstream pickle loading, MuJoCo, dm_control, GUI logging or the ten-million-step driver. The original scientific work is *Learning to Get Up*, SIGGRAPH 2022.
