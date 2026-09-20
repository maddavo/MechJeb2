# Solver Basis V1 reconstruction

This branch reconstructs the MechJeb2 V1 source that produced the verified
successful Solver Basis Downrange Capture DLL.

Target artifact:

- Backup: `MechJeb2 Solver Basis Downrange Capture/20260919-213626/MechJeb2.dll`
- SHA-256: `874D41065996140A883A6623DED9CF8E9BC39E34402D4AF2B56D15412E1DB3B1`
- Installed at 2026-09-19 21:36 Australia/Sydney.
- A landing was reported successful at 2026-09-19 22:22 Australia/Sydney.

The original source was not committed at the time. Reconstruction must replay
the recorded source patches in chronological order through the Solver Basis
patch, commit each coherent checkpoint locally, and exclude later experiments.
No integration or installation may occur until the rebuilt V1 checkpoint has
been reviewed against this target artifact.
