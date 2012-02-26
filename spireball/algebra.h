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

#endif // ALG_Vector_H
