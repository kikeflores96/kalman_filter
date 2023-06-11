#ifndef MATRIX_OPERATIONS_H
#define MATRIX_OPERATIONS_H

#include <vector>

void printMatrix(const std::vector<std::vector<double>>& matrix);
std::vector<std::vector<double>> matrixMultiplication(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B);
void luDecomposition(const std::vector<std::vector<double>>& A, std::vector<std::vector<double>>& L, std::vector<std::vector<double>>& U);
std::vector<double> forwardSubstitution(const std::vector<std::vector<double>>& L, const std::vector<double>& b);
std::vector<double> backwardSubstitution(const std::vector<std::vector<double>>& U, const std::vector<double>& y);
std::vector<std::vector<double>> matrixInverse(const std::vector<std::vector<double>>& A);
std::vector<double> matrixVectorProduct(const std::vector<std::vector<double>>& matrix, const std::vector<double>& vector);
std::vector<std::vector<double>> createDiagMatrix(int size, double diag);
std::vector<std::vector<double>> transposeMatrix(const std::vector<std::vector<double>>& matrix);
std::vector<double> concatenateVectors(const std::vector<double>& v1, const std::vector<double>& v2);
#endif // MATRIX_OPERATIONS_H