#ifndef ALG_Vector_H
#define ALG_Vector_H

/**
 * Translates a point p with vector v
 *
 * p = p + v (p and v are vectors)
 *
 * @param [in,out] p The point (an array of 3 elements)
 * @param [in] v The vector to translate with (an array of 3 elements)
 */
void ALG_translate(double *p, const double *v);
/**
 * Scales a vector.
 *
 * v = cv (c is a scalar, v is a vector)
 *
 * @param [in,out] v The vector to modify (an array of 3 elements).
 * @param [in] c The constant, to scale with.
 */
void ALG_scale(double *v, double c);
/**
 * Calculates the dot product of two vectors
 *
 * returns a*b;
 *
 * @param [in] a,b The two vectors
 *
 * @return Their dot product.
 */
double ALG_dotProduct(const double *a, const double *b);
/**
 * Calculates the cross product of two vectors.
 *
 * result = a×b
 *
 * @param [in,out] result Their cross product. The user must provide storage for the vector. (an array of 3 elements).
 * @param [in] a,b The two vectors (an array of 3 elements).
 *
 */
void ALG_crossProduct(double *result, const double *a, const double *b);
/**
 * Translates a vector with the 3×3 matrix.
 *
 * v = Mv
 *
 * @param [in,out] result  The vector which will store the result (an array of 3 elements)
 * @param [in,out] v The vector to translate (an array of 3 elements).
 * @param [in] matrix The matrix to translate with (The matrix is stored in row major order)
 */
void ALG_transform(double *result, const double *v, const double *matrix);
/**
 * Multiplies two 3×3 matrices.
 *
 * result = AB
 *
 * @param [in,out] result The resulting matrix. The user must supply an array.
 *      (9 element array, the elements are stored in row major order).
 * @param [in] a,b The two matrices to multply (9 element arrays. Elements stored in row major order.)
 */
void ALG_multiplyMatrix(double *result, const double *a, const double *b);
/**
 * Translates the matrix with the given matrix.
 *
 * M = M + T
 *
 * @param [in,out] M The matrix to translate
 * @param [in] T The translator matrix.
 */
void ALG_translateMatrix(double *M, double *T);
/**
 * Scales the matrix with the given factor.
 *
 * M = fM
 *
 * @param [in,out] M The matrix to scale.
 * @param [in] f The scale factor.
 */
void ALG_scaleMatrix(double *M, double f);
/**
 * Resizes the vector to the length of 1. Null vectors remain untouched.
 *
 * Involves a square root and one division.
 *
 * @param [in,out] v The vector to resize
 */
void ALG_normalizeVector(double *v);
/**
 * @param [in] v The vector (which has 3 elements).
 *
 * @return The vector's length.
 *
 * The algorithm needs to calculate the square root of a number.
 * which is slow.
 */
double ALG_getVectorLength(const double *v);
/**
 * Creates a 3×3 matrix that rotates a vector around the given angle.
 *
 * @param [in,out] M The matrix (9 element array). The matrix will be stored in row major order.
 * @param [in] axis The axis to rotate around. The vector's length must be 1 to get the correct results.
 * @param [in] angle The angle of rotation in radians.
 *
 * The algorithm involves the calculation of the sine and cosine of the given angle.
 */
void ALG_createRotationMatrix(double *M, const double *axis, double angle);

/**
 * Returns the vector that point from one point to another.
 *
 * @param [in,out] v 3 element array of the element of the resulting vector.
 * @param [in] a,b The the points (3 element vectors). v will be the vector that points from a to b.
 */
void ALG_getPointToPointVector(double *v, const double *a, const double *b);
/**
 * Transposes a matrix.
 *
 * @param [in,out] M A 3×3 matrix that will be transposed.
 */
void ALG_transposeMatrix(double *M);

/**
 * Solves a linear system of 3 equations. The equations are given in the following form:
 *
 * Ax = b
 *
 * @param [in,out] x Array of 3 elements. It will store the result.
 * @param [in] A The 3×3 matrix of the equation in row major order.
 * @param [in] b The vector of the right side (a vector of 3 elements.)
 *
 * @retval Nonzero if single solution is found.
 * @retval Zero If there is no solution or infinite amount of solutions exist.
 */
int ALG_solveSystem3(double *x, const double *A, const double *b);
/**
 * Calculates the determinant of a matrix.
 *
 * @param [in] M A 3×3 matrix in row major order.
 *
 * @return The determinant.
 */
double ALG_getDeterminant(const double *M);
/**
 * @param [in] v The vector (array of 3 elements)
 *
 * @retval Nonzero if the vector is null vector.
 * @retval Zero otherwise
 */
int ALG_isNullVector(const double *v);
/**
 * Interpolates two arrays
 *
 * @param [in,out] result The resulting array.
 * @param [in] a,b The two arrays to interpolate.
 * @param [in] n The number of elements in the array.
 * @param [in] iFactor The interpolation factor.
 *
 */
void ALG_interpolateArray(double *result, const double *a, const double *b, int n, double iFactor);
/**
 * Multiplies one row with the given factor.
 *
 * @param [in,out] matrix an array with 9 elements.
 * @param [in] rowId Zero-based row id can be 0, 1, 2.
 * @param [in] factor The factor.
 */
void ALG_multiplyRow(double *matrix, char rowId, double factor);
/**
 * Adds the multiple of a row to another
 *
 * @param [in,out] matrix The 3×3 matrix (An array with 9 elements.)
 * @param [in] dstRowId Zero-based index of the row that will be modified.
 * @param [in] sourceRowId Index of the row whose mutiple will be used.
 * @param [in] factor The multipying factor.
 */
void ALG_performRowOperation(double *matrix, char dstRowId, char sourceRowId, double factor);
/**
 * Calculates the inverse of a matrix.
 *
 * @param [in,out] inverse An array of 9 elements that will hold the inverse matrix in row major order.
 * @param [in] matrix The source matrix
 *
 * @retval Non-zero The inverse is calculated.
 * @retval Zero The given matrix is singular and has no inverse.
 */
char ALG_invertMatrix(double *inverse, const double *matrix);

/**
 * Creates a cross product matrix from a vector.
 *
 * a×b = Ab
 *
 * @param [in,out]  cpMatrix 9 element array, will hold the result.
 * @param [in] vector The vector to create the matrix from.
 */
void ALG_createCrossProductMatrix(double *cpMatrix, const double *vector);

#endif // ALG_Vector_H
