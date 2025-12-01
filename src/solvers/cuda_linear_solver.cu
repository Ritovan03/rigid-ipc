#include "cuda_linear_solver.hpp"

#include <cuda_runtime.h>
#include <cusolverSp.h>
#include <cusparse.h>

#include <iostream>

namespace ipc::rigid {

namespace {

// CUDA error checking macros
#define CUDA_CHECK(call) \
    do { \
        cudaError_t error = call; \
        if (error != cudaSuccess) { \
            std::cerr << "CUDA error at " << __FILE__ << ":" << __LINE__ \
                      << " - " << cudaGetErrorString(error) << std::endl; \
            return false; \
        } \
    } while(0)

#define CUSOLVER_CHECK(call) \
    do { \
        cusolverStatus_t status = call; \
        if (status != CUSOLVER_STATUS_SUCCESS) { \
            std::cerr << "cuSOLVER error at " << __FILE__ << ":" << __LINE__ \
                      << " - Status: " << status << std::endl; \
            return false; \
        } \
    } while(0)

#define CUSPARSE_CHECK(call) \
    do { \
        cusparseStatus_t status = call; \
        if (status != CUSPARSE_STATUS_SUCCESS) { \
            std::cerr << "cuSPARSE error at " << __FILE__ << ":" << __LINE__ \
                      << " - Status: " << status << std::endl; \
            return false; \
        } \
    } while(0)

} // anonymous namespace

bool solve_cuda_sparse(
    int n,
    int nnz,
    const double* values,
    const int* row_ptr,
    const int* col_ind,
    const double* b,
    double* x)
{
    // Handle edge cases
    if (n <= 0 || nnz <= 0) {
        return false;
    }

    // Initialize CUDA context
    cusolverSpHandle_t cusolverH = nullptr;
    cusparseHandle_t cusparseH = nullptr;
    cusparseMatDescr_t descrA = nullptr;

    // Device pointers
    double* d_values = nullptr;
    int* d_row_ptr = nullptr;
    int* d_col_ind = nullptr;
    double* d_b = nullptr;
    double* d_x = nullptr;

    try {
        // Create cuSOLVER and cuSPARSE handles
        CUSOLVER_CHECK(cusolverSpCreate(&cusolverH));
        CUSPARSE_CHECK(cusparseCreate(&cusparseH));

        // Create matrix descriptor
        CUSPARSE_CHECK(cusparseCreateMatDescr(&descrA));
        CUSPARSE_CHECK(cusparseSetMatType(descrA, CUSPARSE_MATRIX_TYPE_GENERAL));
        CUSPARSE_CHECK(cusparseSetMatDiagType(descrA, CUSPARSE_DIAG_TYPE_NON_UNIT));
        CUSPARSE_CHECK(cusparseSetMatIndexBase(descrA, CUSPARSE_INDEX_BASE_ZERO));

        // Allocate device memory
        CUDA_CHECK(cudaMalloc(&d_values, nnz * sizeof(double)));
        CUDA_CHECK(cudaMalloc(&d_row_ptr, (n + 1) * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&d_col_ind, nnz * sizeof(int)));
        CUDA_CHECK(cudaMalloc(&d_b, n * sizeof(double)));
        CUDA_CHECK(cudaMalloc(&d_x, n * sizeof(double)));

        // Copy data from host to device
        CUDA_CHECK(cudaMemcpy(d_values, values, nnz * sizeof(double), cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(d_row_ptr, row_ptr, (n + 1) * sizeof(int), cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(d_col_ind, col_ind, nnz * sizeof(int), cudaMemcpyHostToDevice));
        CUDA_CHECK(cudaMemcpy(d_b, b, n * sizeof(double), cudaMemcpyHostToDevice));

        // Initialize solution vector to zero
        CUDA_CHECK(cudaMemset(d_x, 0, n * sizeof(double)));

        // Solve the linear system using QR factorization
        // First, check if the matrix is singular
        int singularity = -1;
        
        // Use the QR solver for general matrices
        cusolverStatus_t solve_status = cusolverSpDcsrlsvqr(
            cusolverH,      // cusolverSp handle
            n,              // number of rows (and columns)
            nnz,            // number of non-zeros
            descrA,         // matrix descriptor
            d_values,       // array of nnz non-zero elements
            d_row_ptr,      // array of n+1 row offsets
            d_col_ind,      // array of nnz column indices
            d_b,            // right-hand side vector
            0.0,            // threshold for singularity detection
            0,              // reordering: 0=no reordering, 1=symrcm, 2=symamd
            d_x,            // solution vector
            &singularity    // -1 if A is invertible
        );

        if (solve_status != CUSOLVER_STATUS_SUCCESS) {
            std::cerr << "cuSOLVER QR solve failed with status: " << solve_status << std::endl;
            if (singularity >= 0) {
                std::cerr << "Matrix is singular at pivot " << singularity << std::endl;
            }
            return false;
        }

        if (singularity >= 0) {
            std::cerr << "Matrix is singular at pivot " << singularity << std::endl;
            return false;
        }

        // Copy solution back to host
        CUDA_CHECK(cudaMemcpy(x, d_x, n * sizeof(double), cudaMemcpyDeviceToHost));

        // Cleanup
        if (d_values) cudaFree(d_values);
        if (d_row_ptr) cudaFree(d_row_ptr);
        if (d_col_ind) cudaFree(d_col_ind);
        if (d_b) cudaFree(d_b);
        if (d_x) cudaFree(d_x);
        
        if (descrA) cusparseDestroyMatDescr(descrA);
        if (cusparseH) cusparseDestroy(cusparseH);
        if (cusolverH) cusolverSpDestroy(cusolverH);

        return true;

    } catch (...) {
        // Cleanup on exception
        if (d_values) cudaFree(d_values);
        if (d_row_ptr) cudaFree(d_row_ptr);
        if (d_col_ind) cudaFree(d_col_ind);
        if (d_b) cudaFree(d_b);
        if (d_x) cudaFree(d_x);
        
        if (descrA) cusparseDestroyMatDescr(descrA);
        if (cusparseH) cusparseDestroy(cusparseH);
        if (cusolverH) cusolverSpDestroy(cusolverH);

        return false;
    }
}

} // namespace ipc::rigid