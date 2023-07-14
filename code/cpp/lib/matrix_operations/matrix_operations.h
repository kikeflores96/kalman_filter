#ifndef MATRIX_OPERATIONS_H
#define MATRIX_OPERATIONS_H
#include <vector>
//---------------------------------------------------------------------------------------
// MATRIX OPERATIONS LIBRARY
//---------------------------------------------------------------------------------------
// Author:  Enrique Flores
// Date:    14/07/2023

// Print a N x N matrix
void printMatrix(const std::vector<std::vector<double>>& matrix);
// Perform LU decomposition of a N x N matrix
void luDecomposition(const std::vector<std::vector<double>>& A, 
                     std::vector<std::vector<double>>& L, 
                     std::vector<std::vector<double>>& U);
// Check whether the a matrix has Inf or Nans
bool hasInfOrNaN(const std::vector<std::vector<double>>& matrix);
// Forward subsitution for matrix inversion
std::vector<double> forwardSubstitution(const std::vector<std::vector<double>>& L, 
                                        const std::vector<double>& b);
// Backward substitution for matrix inversion
std::vector<double> backwardSubstitution(const std::vector<std::vector<double>>& U, 
                                         const std::vector<double>& y);
// Product between a matrix and a column vector
std::vector<double> matrixVectorProduct(const std::vector<std::vector<double>>& matrix, 
                                        const std::vector<double>& vector);
// Concatenate 1D vectors
std::vector<double> concatenateVectors(const std::vector<double>& v1, 
                                       const std::vector<double>& v2);
// Create a diagonal matrix
std::vector<std::vector<double>> createDiagMatrix(int size, double diag);
// Transpose a matrix
std::vector<std::vector<double>> transposeMatrix(
                                 const std::vector<std::vector<double>>& matrix
                                 );
// Multiply two matrices
std::vector<std::vector<double>> matrixMultiplication(
                                 const std::vector<std::vector<double>>& A, 
                                 const std::vector<std::vector<double>>& B
                                 );
// Compute the inverse of a matrix
std::vector<std::vector<double>> matrixInverse(
                                 const std::vector<std::vector<double>>& A
                                 );

#endif // MATRIX_OPERATIONS_H