#pragma once

/**
 * @brief CUDA-based sparse linear solver wrapper.
 * 
 * This header provides an interface for solving sparse linear systems
 * on GPU using NVIDIA cuSOLVER library.
 */

namespace ipc::rigid {

/**
 * @brief Solve a sparse linear system Ax = b using CUDA cuSOLVER.
 * 
 * This function uses cuSOLVER's sparse Cholesky factorization to solve
 * the linear system. The input matrix should be symmetric positive definite
 * (or semi-definite with regularization).
 * 
 * @param[in] n         Size of the matrix (n x n)
 * @param[in] nnz       Number of non-zero entries in the matrix
 * @param[in] values    Array of non-zero values (CSR format)
 * @param[in] row_ptr   Array of row pointers (size n+1, CSR format)
 * @param[in] col_ind   Array of column indices (size nnz, CSR format)
 * @param[in] b         Right-hand side vector (size n)
 * @param[out] x        Solution vector (size n)
 * 
 * @return true if the solve was successful, false otherwise
 * 
 * @note The matrix data is expected in CSR (Compressed Sparse Row) format.
 *       Since Eigen uses CSC format but the Hessian is symmetric, the CSC
 *       representation of H is equivalent to CSR representation of H^T = H.
 */
bool solve_cuda_sparse(
    int n,
    int nnz,
    const double* values,
    const int* row_ptr,
    const int* col_ind,
    const double* b,
    double* x);

} // namespace ipc::rigid