// wrapper.cpp
#include "wrapper.h"
#include "Eigen/Dense"
#include <cmath>
#include <cstring>

using namespace Eigen;

extern "C" {
    void multiply_matrices(const double* a, const double* b, double* result, int rows, int cols) {
        Map<const MatrixXd> matA(a, rows, cols);
        Map<const MatrixXd> matB(b, cols, rows);
        MatrixXd matC = matA * matB;
        Map<MatrixXd>(result, rows, rows) = matC;
    }
}
