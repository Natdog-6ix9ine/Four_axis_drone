// wrapper.h
#ifndef WRAPPER_H
#define WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

void multiply_matrices(const double* a, const double* b, double* result, int rows, int cols);

#ifdef __cplusplus
}
#endif

#endif // WRAPPER_H
