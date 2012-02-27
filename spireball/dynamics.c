#include "dynamics.h"
#include "algebra.h"

/**
 * Computes the moment of intertia for the given axis.
 *
 * @param [in] tensor 9 element array of the inertia tensor matrix in row major order.
 * @param [in] axis 3 element array of the axis vector in object space (so if
 *      the body is rotated the vector must be transformed too.). The length of
 *      the axis vector must be 1.
 *
 * @return The moment of intertia around the given axis.
 */
double computeMomentOfInertia(const double *tensor, const double *axis)
{
    double transformed[3];
    ALG_transform(transformed, axis, tensor);
    return ALG_dotProduct(transformed, axis);
}

/**
 * Moves the body using its attributes.
 *
 * @param [in,out] body The body to move.
 * @param [in] timeStep The time interval.
 */
void moveBody(DYN_Body *body, double timeStep)
{
    // CONTINUE HERE!
}
