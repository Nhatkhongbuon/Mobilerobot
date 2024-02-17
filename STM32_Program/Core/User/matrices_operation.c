#include "matrices_op2.h"

//Allocate memmory space and matrix A
void allocate_matrix(matrix *A, unsigned_int8 num_rows, unsigned_int8 num_columns)
{
    A->index = (double **) malloc(sizeof(double *) * num_rows);
    A->num_columns = num_columns;
    A->num_rows = num_rows;

    for (int i = 0; i < A->num_rows; i++)
        A->index[i] = (double *) calloc(sizeof(double), A->num_columns);
}

//Deallocate memory space for matrix A
void deallocate_matrix(matrix *A)
{
    for (int i = 0; i < A->num_rows; i++){
        free(A->index[i]);
    }
    free(A->index);
}

//Change dimensions of matrix A
void reallocate_matrix(matrix *A, unsigned_int8 num_rows, unsigned_int8 num_columns)
{
    deallocate_matrix(A);
    allocate_matrix(A, num_rows, num_columns);
}

//Calculate addtion of matrix A and B
//Answer is directly assigned to A
void addition(matrix *A, matrix *B)
{
    //Check dimensions of all matrix operands
    if ((A->num_rows != B->num_rows) || (A->num_columns != B->num_columns))
        return;
    
    //Calculate addition
    for (int i  = 0; i < A->num_rows; i++){
        for (int j = 0; j < A->num_columns; j++){
            A->index[i][j] += B->index[i][j];
        }
    }
}

//Calculate subtraction of matrix A and B (A - B)
//Answer is directly assigned to A
void subtraction(matrix *A, matrix *B)
{
    //Check dimensions of all matrix operands
    if ((A->num_rows != B->num_rows) || (A->num_columns != B->num_columns))
        return;
    
    //Calculate Subtraction
    for (int i  = 0; i < A->num_rows; i++){
        for (int j = 0; j < A->num_columns; j++){
            A->index[i][j] -= B->index[i][j];
        }
    }
}

//Scalar multiplication of matrix
////Answer is directly assigned to A
void scalar_multiplication(matrix *A, double scalar)
{
    for (int i  = 0; i < A->num_rows; i++){
        for (int j = 0; j < A->num_columns; j++){
            A->index[i][j] *= scalar;
        }
    }
}

//Calculate multiplication of matrix A and B
//Answer is directly assigned to Ans
void mutiplication(matrix *A, matrix *B, matrix *Ans)
{
    //Check for appropriate size
    if (A->num_columns != B->num_rows)
        return;

    //Allocate memory space for answer
    allocate_matrix(Ans, A->num_rows, B->num_columns);

    //Perform multiplication
    for (int i = 0; i < Ans->num_rows; i++){
        for(int j = 0; j < Ans->num_columns; j++){
            for (int x = 0; x < A->num_columns; x++){
                Ans->index[i][j] += (A->index[i][x] * B->index[x][j]); 
            }
        }
    }
}

//Perform transpose of A
void transpose(matrix *A, matrix *transpose_A)
{
    allocate_matrix(transpose_A, A->num_columns, A->num_rows);

    //Perform transpose operation
    for (int i = 0; i < transpose_A->num_rows; i++){
        for (int j = 0; j < transpose_A->num_columns; j++){
            transpose_A->index[i][j] = A->index[j][i];
        }
    }
}

void minor(matrix *major, matrix *minor, unsigned_int8 skip_row, unsigned_int8 skip_column)
{
    //Allocate memory space for minor
    allocate_matrix(minor, major->num_rows - 1, major->num_columns - 1);

    int x = 0;
    int y = 0;

    //Assign element to minor
    //Skip row skip_row
    //Skip column skip_column
    for (int i = 0; i < major->num_rows; i++){
        if (i != skip_row){
            for (int j = 0; j < major->num_columns; j++){
                if (j != skip_column){
                    minor->index[x][y] = major->index[i][j];
                    y++;
                } else {
                    continue;
                }
            }
            x++;
            y = 0;
        } else {
            continue;
        }
    }
}

// Calculate the determinant of A
double determinant(matrix *A, unsigned_int8 expand_row)
{
    double result = 0;
    
    if (A->num_rows == 1 && A->num_columns == 1)
        return result = A->index[0][0];

    //Calculate determinant with chosen expand_row
    for (int j = 0; j < A->num_columns; j++){
        
        //Create minor matrix
        matrix M;
        minor(A, &M, expand_row, j);
        
        if ((expand_row + j) % 2 == 0){
            result += (A->index[expand_row][j] == 0) ? (0) : (A->index[expand_row][j] * determinant(&M, 0));
        } else {
            result += (A->index[expand_row][j] == 0) ? (0) : ((-1) * A->index[expand_row][j] * determinant(&M, 0));
        }

        //Deallocate minor matrix
        deallocate_matrix(&M);
    }

    return result;
}

//Calculate adjoint matrix
void adjoint(matrix *A, matrix *Ans)
{
    matrix temp;
    allocate_matrix(&temp, A->num_rows, A->num_columns);
    for (int i = 0; i < temp.num_rows; i++){
        for (int j = 0; j < temp.num_columns; j++){
            matrix M;
            minor(A, &M, i, j);

            temp.index[i][j] = ((i + j) % 2 == 0) ? (determinant(&M, 0)) : ((-1) * determinant(&M, 0));
            
            //Deallocate minor matrix
            deallocate_matrix(&M);
        }
    }
    transpose(&temp, Ans);
    deallocate_matrix(&temp);
}

void inverse(matrix *A, matrix *inverse_of_A)
{
    //Calculate determinant of A
    double determinant_of_A = determinant(A, 0);

    //Calculate adjoint matrix of A
    adjoint(A, inverse_of_A);

    //Calculate inverse of A
    scalar_multiplication(inverse_of_A, 1 / determinant_of_A);
}
