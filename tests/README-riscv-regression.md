# RISC-V regression tests (PR #29)

- `crash-repro-8x8-riscv.264` — minimal High/CABAC/8x8-transform clip
  reproducing the misaligned-access crash fixed in commit 1
  ("riscv: fix misaligned-trap crashes and harden the slice loop").

- `riscv-scalar-fastpaths.264` + `.golden` — static scene + synthetic
  pan, High/CABAC/8x8, exercising the scalar deblock and motion
  compensation fast paths added in commits 2-3. The `.golden` file is
  one FNV-1a hash per output frame, recorded from an unmodified x86
  SIMD build (backend-independent ground truth).

- `riscv_regression_check.c` — ~150-line, dependency-free harness
  (only edge264.h + libc) to decode a clip and either print or check
  per-frame hashes against a golden file:

  ```sh
  cc -O2 -I.. -o riscv_regression_check riscv_regression_check.c ../src/edge264.c -lpthread -lm
  ./riscv_regression_check tests/riscv-scalar-fastpaths.264 0 tests/riscv-scalar-fastpaths.golden
  ```

  Exits non-zero with the offending frame index and both hashes on any
  mismatch — intended as a lightweight guard for the fast paths added
  in this PR against future refactors (e.g. the planned multithreading
  rework), independent of which backend/build runs it.
