# OpenMP Porting Plan for Rigid-Body IPC Simulation

This document describes a clear, actionable plan to add OpenMP-based CPU parallelism to the rigid-body IPC simulation. The goal is to use multiple CPU cores to speed up the hottest code paths while keeping data on the host (no PCIe transfers).

Primary targets (by expected impact):

1. Sparse linear solve (Newton step)
2. Hessian & gradient assembly
3. Narrow-phase collision detection (CCD)

---

## 1. Sparse Linear Solve (Primary Bottleneck)

Location

-   Target file: `src/solvers/newton_solver.cpp`
-   Target function: `NewtonSolver::compute_direction` (approx. lines ~265–280)

Problem

Eigen's `SimplicialLDLT` is single-threaded. Simply adding `#pragma omp` will not parallelize the solver. To use multiple CPU cores you should switch to a parallel solver implementation (e.g., Intel MKL's Pardiso via Eigen's Pardiso support) or use a threaded linear algebra library.

OpenMP / MKL strategy

-   Replace the solver type with `Eigen::PardisoLDLT<Eigen::SparseMatrix<double>>` (requires MKL and Eigen's Pardiso support).
-   Reuse the same analyze/factorize/solve workflow to minimize code changes.

Example change

```cpp
// include
#include <Eigen/PardisoSupport>

// inside compute_direction
Eigen::PardisoLDLT<Eigen::SparseMatrix<double>> linear_solver;
linear_solver.analyzePattern(hessian);
linear_solver.factorize(hessian);
direction = linear_solver.solve(-gradient);
```

Build requirements

-   Link against Intel MKL (or another threaded sparse solver such as MUMPS).
-   Enable OpenMP in the compiler flags (see Section 4).

Notes

-   If the sparsity pattern is static, preserve symbolic analysis between iterations to reduce cost.
-   If MKL isn't available, consider parallel sparse direct solvers or iterative solvers with threaded mat-vec and preconditioning.

---

## 2. Hessian & Gradient Assembly

Location

-   `src/problems/distance_barrier_rb_problem.cpp`
-   Key functions: `compute_energy_term`, `compute_barrier_term`, `compute_friction_term`

Strategy

-   Use a map-reduce pattern with thread-local buffers to avoid locking inside the hot loop. Each thread computes its local gradient and triplet list; a short merge phase combines results.

Why this works

-   The heavy geometry/math per-constraint is parallelizable with no shared state. Merging per-thread results once is much cheaper than repeated synchronization inside the inner loop.

Example pattern

```cpp
// global containers
std::vector<Eigen::Triplet<double>> global_triplets;
Eigen::VectorXd global_gradient = Eigen::VectorXd::Zero(num_vars);

#pragma omp parallel
{
    std::vector<Eigen::Triplet<double>> local_triplets;
    Eigen::VectorXd local_gradient = Eigen::VectorXd::Zero(num_vars);

    #pragma omp for nowait
    for (int i = 0; i < (int)constraints.size(); ++i) {
        // compute local small Hessian block and gradient contribution
        // local_triplets.push_back(...);
        // local_gradient += ...;
    }

    #pragma omp critical
    {
        global_triplets.insert(global_triplets.end(),
                               local_triplets.begin(), local_triplets.end());
        global_gradient += local_gradient;
    }
}
```

Best practices

-   Preallocate per-thread buffers where possible (avoid repeated allocations).
-   Use `omp_get_max_threads()` to size per-thread containers if you prefer indexing by thread ID instead of critical sections.
-   If memory is constrained, merge in batches to limit peak memory use.

---

## 3. Narrow-Phase Collision Detection (CCD)

Location

-   `src/problems/distance_barrier_rb_problem.cpp` (calls into `ccd/rigid/time_of_impact.cpp`)

Strategy

-   Use an OpenMP parallel-for with a `reduction(min: earliest_toi)` clause to compute the global minimum Time of Impact (TOI) across candidate pairs.

Example

```cpp
double earliest_toi = 1.0;

#pragma omp parallel for reduction(min:earliest_toi) schedule(dynamic)
for (int i = 0; i < (int)candidate_pairs.size(); ++i) {
    double toi = compute_edge_edge_time_of_impact(candidate_pairs[i]);
    if (toi < earliest_toi) earliest_toi = toi;
}

return earliest_toi;
```

Notes

-   Use `schedule(dynamic)` to balance variable-cost CCD queries.
-   Consider grouping candidate pairs by type to reduce branch divergence and improve instruction locality.

---

## 4. Required libraries & build settings

Compiler flags

-   GCC/Clang: add `-fopenmp` to compile and link flags.
-   CMake (recommended):

```cmake
find_package(OpenMP REQUIRED)
target_link_libraries(your_target PUBLIC OpenMP::OpenMP_CXX)
```

Intel MKL (Pardiso)

-   If using MKL Pardiso via Eigen, link MKL to your target. Exact flags depend on your MKL installation and threading model. Example (adjust for your platform and MKL variant):

```cmake
# Example: link with MKL (Intel-provided targets or manual flags)
target_link_libraries(your_target PUBLIC mkl_intel_lp64 mkl_intel_thread mkl_core pthread)
target_compile_definitions(your_target PUBLIC -DINTEL_MKL)
```

-   A more portable approach is to use `FindMKL.cmake` or Intel's `mkl#/cmake` modules. If MKL is unavailable, MUMPS or other parallel solvers are alternatives.

Disable TBB

-   Remove or guard existing TBB usage (e.g., `tbb::parallel_for`, `tbb::concurrent_vector`) to avoid runtime conflicts. Choose one threading runtime for clarity.

---

## 5. Testing and validation

-   Start with small deterministic scenes and compare CPU (single-threaded) vs OpenMP runs for energy, gradient, Hessian entries, and simulation output.
-   Add unit tests for per-constraint contributions and global assembly.
-   Measure scaling (runtime vs threads) to ensure the expected speedup.

---

## 6. Incremental rollout / roadmap

1. Replace TBB-based assembly with the map-reduce OpenMP pattern (per-constraint parallel loops + thread-local buffers). Validate correctness.
2. Integrate a threaded sparse solver (MKL Pardiso) in `NewtonSolver::compute_direction`. Provide a build option to switch between Eigen sequential and MKL.
3. Parallelize CCD with `#pragma omp parallel for reduction(min:...)`.
4. Profile and optimize hotspots (memory bandwidth, false sharing, allocation hotspots).

---

## 7. Final notes

-   OpenMP is a low-risk, high-reward approach for CPU parallelism in this codebase. It keeps the data on the host and avoids the large engineering cost of a GPU port.
-   I can generate concrete diffs for the three target areas (`distance_barrier_rb_problem.cpp`, `newton_solver.cpp`, and `ccd/time_of_impact.cpp`) and a sample `CMakeLists.txt` patch to add OpenMP and optional MKL — tell me which one you'd like first.

_Last updated: OpenMP porting plan._
