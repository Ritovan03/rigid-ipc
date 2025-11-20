# CUDA Porting Plan for Rigid-Body IPC Simulation

This document describes a pragmatic plan to port the most computationally expensive parts of the rigid-body IPC simulation to CUDA. The primary goals are:

-   Offload heavy linear algebra and per-constraint computations to the GPU.
-   Keep data resident on the device to avoid frequent PCIe transfers.
-   Replace CPU-only libraries (Eigen/TBB) with GPU-friendly alternatives where needed.

Targets prioritized by expected impact:

1. Sparse linear solve (Newton step) — highest impact
2. Hessian & gradient assembly — required to make the solver efficient on the GPU
3. Narrow-phase collision detection (CCD) — good candidate for GPU parallelism

---

## 1. Sparse Linear Solve (Primary Bottleneck)

Location

-   Target file: `src/solvers/newton_solver.cpp`
-   Target function: `NewtonSolver::compute_direction` (approx. lines ~265–280)

Current (CPU) implementation

```cpp
Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> linear_solver;
linear_solver.analyzePattern(hessian);
linear_solver.factorize(hessian);
direction = linear_solver.solve(-gradient);
```

CUDA implementation strategy

-   Library choices: `cuSOLVER` (sparse solvers) or `cuSPARSE` + `cuSOLVER` wrappers.
-   Workflow:
    1.  Ensure the global Hessian matrix `H` and gradient vector `g` live in GPU memory.
    2.  Convert the Hessian to CSR format (or keep it in CSR on the GPU after assembly).
    3.  Use appropriate solver calls, e.g. `cusolverSpDcsrlsvqr` (QR) or `cusolverSpDcsrlsvchol` (Cholesky), depending on matrix properties.
    4.  Keep the result `Δx` on the device if subsequent steps operate on the GPU; copy back only when needed.

Notes

-   Use GPU-side factorization where possible and reuse symbolic analysis if the sparsity pattern is constant across iterations (to reduce overhead).
-   For iterative solvers (if you choose them), consider `cuSPARSE`'s iterative preconditioned solvers; these can be cheaper when exact factorization is expensive.

---

## 2. Hessian & Gradient Assembly

Motivation

Assembling `H` and `g` on the CPU and then copying them to the GPU every Newton step creates heavy PCIe traffic and will likely negate solver speedups. Assemble on the GPU instead.

Location

-   `src/problems/distance_barrier_rb_problem.cpp`
-   Key functions: `compute_energy_term`, `compute_barrier_term`, `compute_friction_term`

CUDA implementation strategy

-   Kernel design:

    -   Each CUDA thread (or small cooperative tile) computes the contribution of a single constraint or rigid body.
    -   If a constraint contributes a small dense block to `H`, compute that block in shared memory then write to global buffers.

-   Data structures:

    -   Create POD GPU-compatible structs for rigid bodies and constraints. Avoid `std::vector`/Eigen types in device code.
    -   Use `thrust::device_vector` or raw device buffers for arrays.

-   Accumulation and assembly:

    -   Gradient: use `atomicAdd` for double (or use block/local reduction + atomicAdd) to accumulate into the global gradient vector.
    -   Hessian: have threads write triplets `(row, col, value)` into pre-allocated device buffers. After all triplets are emitted:
        1. Sort triplets by `(row, col)` using Thrust.
        2. Reduce by key to sum duplicates and form CSR arrays (rowPtr, colIdx, values) on the GPU.

-   Alternatives:
    -   If GPU-side CSR assembly is too complex initially, emit triplets to GPU memory and then transfer the compact triplet list to the host for assembly with Eigen. This reduces copies compared to fully host-side generation but still has a transfer cost.

Best practices

-   Reuse buffers across iterations to avoid repeated allocations.
-   Precompute sparsity pattern (if static) and only update values during runtime; this allows reuse of symbolic analysis for solvers.

---

## 3. Narrow-Phase Collision Detection (CCD)

Motivation

CCD computes time-of-impact (TOI) per candidate pair (edge-edge, face-vertex). Each pair is independent — a natural GPU target.

Location

-   `src/ccd/rigid/time_of_impact.cpp` (used by `distance_barrier_rb_problem.cpp`)

CUDA implementation strategy

-   Move the candidate pair arrays to the GPU and launch a kernel where each thread processes one pair.
-   Port helper functions like `compute_edge_edge_time_of_impact` and `compute_face_vertex_time_of_impact` to `__device__` functions.
-   Store TOI results in a device array and reduce with `thrust::min_element` or a custom reduction to find the global minimum.

Notes

-   Be careful with branch divergence: group candidate pairs by type if possible (edge-edge vs face-vertex).
-   Use double precision if simulation requires; otherwise check numerical stability in single precision.

---

## 4. Data layout & memory strategy

-   Keep primary data on the device (positions, velocities, constraints) as long as simulation logic permits.
-   Minimize host-device roundtrips. For example, keep the Newton iterate, gradient, and Hessian on the GPU through the whole solve step.
-   Reuse device buffers across iterations. Preallocate big arrays for triplets and shrink logically by tracking counts on the device.

---

## 5. Autodiff and third-party libraries

-   Eigen and many CPU AD libraries are not CUDA-compatible. Options:
    -   Implement hand-coded derivatives for the hot loops (often simpler and faster on GPU).
    -   Use GPU-capable math libraries (Thrust, cuBLAS, cuSPARSE) and consider integrating Enzyme for CUDA if you want an automated AD path.

---

## 6. Testing, validation, and correctness

-   Start with small, deterministic problems. Compare CPU and GPU results (energy/gradient/Hessian) to machine precision or acceptable tolerance.
-   Add unit tests for:
    -   Per-constraint energy/grad/Hessian kernels (golden reference from CPU implementation).
    -   Sparse solver correctness with random but well-conditioned matrices.

---

## 7. Minimal example & commands

Below is a minimal iteration flow (pseudo-commands) to test a GPU Hessian assembly and solve loop. Adapt to your build system.

```bash
# Configure CMake with CUDA enabled (example)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUSE_CUDA=ON
cmake --build build -j

# Run a small problem and compare outputs
./build/rigid_ipc_sim --test-gpu-hessian
```

The project will need CMake modifications to add CUDA sources and link against `cudart`, `cusparse`, and `cusolver`.

---

## 8. Roadmap and recommended incremental approach

1. Implement GPU triplet emission kernel for per-constraint Hessian blocks. Keep the rest CPU-side and transfer triplets for assembly.
2. Implement GPU-side sorting/reduction (Thrust) and form a CSR matrix on the device.
3. Replace the Eigen sparse solve with `cuSOLVER` or a `cuSPARSE`-based solver, keeping everything device-resident.
4. Port CCD kernels.
5. Optimize memory layout and try iterative solvers or preconditioners if direct factorization is too expensive.

---

## 9. Final notes

-   Porting is a non-trivial engineering effort. Start with a small, well-instrumented baseline and iterate.
-   I can help generate example kernels, CMake edits, or a prototype for one of the three target areas — tell me which you'd like me to implement first.

---

_Last updated: consolidated CUDA porting plan._
