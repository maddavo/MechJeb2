# Solver Basis V1 reconstruction

This branch reconstructs the MechJeb2 V1 source that produced the verified
successful Solver Basis Downrange Capture DLL.

Target artifact:

- Backup: `MechJeb2 Predictor Closed-Loop Deorbit/20260920-005719/MechJeb2.dll`
- SHA-256: `7320BB280ECF082AEA3DD3FF8C22FAC836A5BE930B57B08F27C7D0166936D049`
- Installed at 2026-09-19 21:36 Australia/Sydney.
- A landing was reported successful at 2026-09-19 22:22 Australia/Sydney.

The original source was not committed at the time. `decompiled-solver-basis-actual/`
is the persistent generated C# reference recovered from this exact target DLL.
Reconstruction must use it with the recorded chronological patches, commit each
coherent checkpoint locally, and exclude later experiments.
