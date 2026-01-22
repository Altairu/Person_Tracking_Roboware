#ifndef MATRIX_H
#define MATRIX_H

#include <string.h>
#include <math.h>

// Tiny Matrix Library for EKF (Fixed size 7x7 max for now)
#define MAX_DIM 7

typedef struct {
    int rows;
    int cols;
    float data[MAX_DIM][MAX_DIM];
} mat_t;

// Init matrix
static inline void mat_init(mat_t *m, int r, int c, float val) {
    m->rows = r; m->cols = c;
    for(int i=0;i<r;i++) for(int j=0;j<c;j++) m->data[i][j] = (i==j) ? val : 0.0f;
}

// Zero matrix
static inline void mat_zero(mat_t *m, int r, int c) {
    m->rows = r; m->cols = c;
    memset(m->data, 0, sizeof(m->data));
}

// Matrix Addition: C = A + B
static inline void mat_add(mat_t *C, const mat_t *A, const mat_t *B) {
    C->rows = A->rows; C->cols = A->cols;
    for(int i=0;i<A->rows;i++) for(int j=0;j<A->cols;j++) C->data[i][j] = A->data[i][j] + B->data[i][j];
}

// Matrix Subtraction: C = A - B
static inline void mat_sub(mat_t *C, const mat_t *A, const mat_t *B) {
    C->rows = A->rows; C->cols = A->cols;
    for(int i=0;i<A->rows;i++) for(int j=0;j<A->cols;j++) C->data[i][j] = A->data[i][j] - B->data[i][j];
}

// Matrix Multiplication: C = A * B
static inline void mat_mul(mat_t *C, const mat_t *A, const mat_t *B) {
    C->rows = A->rows; C->cols = B->cols;
    for(int i=0;i<C->rows;i++) {
        for(int j=0;j<C->cols;j++) {
            C->data[i][j] = 0.0f;
            for(int k=0;k<A->cols;k++) C->data[i][j] += A->data[i][k] * B->data[k][j];
        }
    }
}

// Matrix Transpose: dest = src^T
static inline void mat_trans(mat_t *dest, const mat_t *src) {
    dest->rows = src->cols; dest->cols = src->rows;
    for(int i=0;i<src->rows;i++) for(int j=0;j<src->cols;j++) dest->data[j][i] = src->data[i][j];
}

// Matrix Inversion (Gauss-Jordan, simple implementation for small size)
// Returns 0 on success, -1 on singular
static inline int mat_inv(mat_t *dest, const mat_t *src) {
    int n = src->rows;
    if (n != src->cols) return -1;
    dest->rows = n; dest->cols = n;
    
    // Augmented matrix [A | I]
    float aug[MAX_DIM][2*MAX_DIM];
    for(int i=0;i<n;i++) {
        for(int j=0;j<n;j++) aug[i][j] = src->data[i][j];
        for(int j=n;j<2*n;j++) aug[i][j] = (i==(j-n)) ? 1.0f : 0.0f;
    }

    for(int i=0;i<n;i++) {
        // Pivot
        float pivot = aug[i][i];
        if(fabsf(pivot) < 1e-9) return -1;
        
        for(int j=0;j<2*n;j++) aug[i][j] /= pivot;
        
        for(int k=0;k<n;k++) {
            if(k!=i) {
                float factor = aug[k][i];
                for(int j=0;j<2*n;j++) aug[k][j] -= factor * aug[i][j];
            }
        }
    }
    
    // Copy result
    for(int i=0;i<n;i++) for(int j=0;j<n;j++) dest->data[i][j] = aug[i][j+n];
    return 0;
}

#endif
