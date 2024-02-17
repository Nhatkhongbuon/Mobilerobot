
#ifndef USER_MATRICES_OP2_H_
#define USER_MATRICES_OP2_H_

#include <stdlib.h>
#include <math.h>

//1 byte integer
typedef unsigned char unsigned_int8;

//Structure of matrix
struct MATRIX_struct
{
    double **index;
    unsigned_int8 num_columns;
    unsigned_int8 num_rows;
};

//Define matrix type
typedef struct MATRIX_struct matrix;

//Standard matrices operator
void allocate_matrix(matrix *A, unsigned_int8 num_rows, unsigned_int8 num_columns);
void deallocate_matrix(matrix *A);
void reallocate_matrix(matrix *A, unsigned_int8 num_rows, unsigned_int8 num_columns);
void addition(matrix *A, matrix *B);
void subtraction(matrix *A, matrix *B);
void scalar_multiplication(matrix *A, double scalar);
void mutiplication(matrix *A, matrix *B, matrix *Ans);
void transpose(matrix *A, matrix *transpose_A);
void inverse(matrix *A, matrix *inverse_of_A);
void adjoint(matrix *A, matrix *Ans);
double determinant(matrix *A, unsigned_int8 expand_row);
void minor(matrix *major, matrix *minor, unsigned_int8 skip_row, unsigned_int8 skip_column);


#endif /* USER_MATRICES_OP2_H_ */
