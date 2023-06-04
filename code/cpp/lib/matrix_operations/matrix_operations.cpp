#include <vector>
#include <iostream>
#include <cmath>

void printMatrix(const std::vector<std::vector<double>>& matrix) {
    for (const auto& row : matrix) {
        for (const auto& element : row) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
}



std::vector<std::vector<double>> matrixMultiplication(const std::vector<std::vector<double>>& A, const std::vector<std::vector<double>>& B) {
    int n = A.size();
    std::vector<std::vector<double>> C(n, std::vector<double>(n, 0.0));

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < n; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }

    return C;
}


// Function to perform LU decomposition of a matrix
void luDecomposition(const std::vector<std::vector<double>>& A, std::vector<std::vector<double>>& L, std::vector<std::vector<double>>& U) {
    int n = A.size();

    // Initialize L and U matrices
    L = std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0));
    U = std::vector<std::vector<double>>(n, std::vector<double>(n, 0.0));

    for (int i = 0; i < n; i++) {
        L[i][i] = 1.0; // Diagonal elements of L are 1

        for (int j = 0; j < n; j++) {
            if (i <= j) {
                // U matrix (upper triangular part)
                U[i][j] = A[i][j];
                for (int k = 0; k < i; k++) {
                    U[i][j] -= L[i][k] * U[k][j];
                }
            } else {
                // L matrix (lower triangular part)
                L[i][j] = A[i][j];
                for (int k = 0; k < j; k++) {
                    L[i][j] -= L[i][k] * U[k][j];
                }
                L[i][j] /= U[j][j];
            }
        }
    }
}

// Function to solve the system of linear equations Ly = b
std::vector<double> forwardSubstitution(const std::vector<std::vector<double>>& L, const std::vector<double>& b) {
    int n = L.size();
    std::vector<double> y(n, 0.0);

    for (int i = 0; i < n; i++) {
        y[i] = b[i];
        for (int j = 0; j < i; j++) {
            y[i] -= L[i][j] * y[j];
        }
    }

    return y;
}

// Function to solve the system of linear equations Ux = y
std::vector<double> backwardSubstitution(const std::vector<std::vector<double>>& U, const std::vector<double>& y) {
    int n = U.size();
    std::vector<double> x(n, 0.0);

    for (int i = n - 1; i >= 0; i--) {
        x[i] = y[i];
        for (int j = i + 1; j < n; j++) {
            x[i] -= U[i][j] * x[j];
        }
        x[i] /= U[i][i];
    }

    return x;
}

// Function to invert a matrix using LU decomposition
std::vector<std::vector<double>> matrixInverse(const std::vector<std::vector<double>>& A) {
    int n = A.size();
    std::vector<std::vector<double>> L, U;
    luDecomposition(A, L, U);

    std::vector<std::vector<double>> inverse(n, std::vector<double>(n, 0.0));
    std::vector<double> b(n, 0.0);

    for (int i = 0; i < n; i++) {
        b[i] = 1.0;
        std::vector<double> y = forwardSubstitution(L, b);
        std::vector<double> x = backwardSubstitution(U, y);
        for (int j=0; j<n; j++){
            inverse[j][i] = x[j];
        }
        
        b[i] = 0.0;
    }

    return inverse;
}



// Function to perform matrix-vector multiplication
std::vector<double> matrixVectorProduct(const std::vector<std::vector<double>>& matrix, const std::vector<double>& vector) {
    int n = matrix.size();
    std::vector<double> result(n, 0.0);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            result[i] += matrix[i][j] * vector[j];
        }
    }

    return result;
}