#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include <utils/eigen_ext.hpp>

namespace ipc::rigid {

class OptimizationProblem {
public:
    virtual ~OptimizationProblem() = default;

    /// Compute the objective function f(x)
    virtual double compute_objective(
        const Eigen::VectorXd& x,
        Eigen::VectorXd& grad,
        Eigen::SparseMatrix<double>& hess,
        bool compute_grad = true,
        bool compute_hess = true) = 0;

    // --------------------------------------------------------------------
    // Convience functions
    // --------------------------------------------------------------------

    virtual double compute_objective(const Eigen::VectorXd& x) final
    {
        Eigen::VectorXd grad;
        Eigen::SparseMatrix<double> hess;
        return compute_objective(
            x, grad, hess,
            /*compute_grad=*/false, /*compute_hess=*/false);
    }

    virtual double
    compute_objective(const Eigen::VectorXd& x, Eigen::VectorXd& grad) final
    {
        Eigen::SparseMatrix<double> hess;
        return compute_objective(
            x, grad, hess,
            /*compute_grad=*/true, /*compute_hess=*/false);
    }

    virtual void update_augmented_lagrangian(const Eigen::VectorXd& x) {}
    virtual bool
    are_equality_constraints_satisfied(const Eigen::VectorXd& x) const
    {
        return true;
    }

    /// @returns the number of variables
    virtual int num_vars() const = 0;

    /// @returns A vector of booleans indicating if a DoF is fixed.
    virtual const VectorXb& is_dof_fixed() const = 0;

    virtual Eigen::VectorXi free_dof() const
{
    const VectorXb& is_fixed = is_dof_fixed();
    long n_total = is_fixed.size();
    long n_free = n_total - is_fixed.count();
    Eigen::VectorXi free_dof(n_free);

    // If data size is small, use serial implementation to avoid thread overhead
    if (n_total < 10000) {
        for (int i = 0, j = 0; i < n_total; i++) {
            if (!is_fixed(i)) {
                free_dof(j++) = i;
            }
        }
        return free_dof;
    }

    int num_threads = omp_get_max_threads();
    std::vector<int> offsets(num_threads + 1, 0);

    #pragma omp parallel
    {
        int tid = omp_get_thread_num();
        int count = 0;
        #pragma omp for schedule(static)
        for (int i = 0; i < n_total; ++i) {
            if (!is_fixed(i)) count++;
        }
        offsets[tid + 1] = count;
    }

    // Compute Prefix Sum to determine start indices for each thread
    for (int i = 0; i < num_threads; ++i) {
        offsets[i + 1] += offsets[i];
    }

    #pragma omp parallel
    {
        int tid = omp_get_thread_num();
        int write_idx = offsets[tid];
        
        #pragma omp for schedule(static)
        for (int i = 0; i < n_total; ++i) {
            if (!is_fixed(i)) {
                free_dof(write_idx++) = i;
            }
        }
    }

    return free_dof;
}

    /// Determine if there is a collision between two configurations
    virtual bool
    has_collisions(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj) = 0;

    /// Compute the earliest time of impact between two configurations
    virtual double compute_earliest_toi(
        const Eigen::VectorXd& xi, const Eigen::VectorXd& xj) = 0;

    virtual bool is_ccd_aligned_with_newton_update() = 0;

    /// Compute the minimum distance among geometry
    virtual double compute_min_distance(const Eigen::VectorXd& x) const = 0;

    /// Get the world coordinates of the vertices
    virtual Eigen::MatrixXd world_vertices(const Eigen::VectorXd& x) const = 0;

    /// Get the length of the diagonal of the worlds bounding box
    virtual double world_bbox_diagonal() const = 0;

    /// Get the mass matrix
    virtual DiagonalMatrixXd mass_matrix() const = 0;

    /// Get the average mass
    virtual double average_mass() const = 0;

    /// Get the time-step
    virtual double timestep() const = 0;

    virtual bool is_barrier_problem() const { return false; }
    virtual bool is_constrained_problem() const { return false; }
};

/// Helper Functions for checking finite differences
/// ------------------------------------------------
Eigen::VectorXd eval_grad_objective_approx(
    OptimizationProblem& problem, const Eigen::VectorXd& x);
Eigen::MatrixXd eval_hess_objective_approx(
    OptimizationProblem& problem, const Eigen::VectorXd& x);

} // namespace ipc::rigid
