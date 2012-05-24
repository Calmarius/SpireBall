/**
 * Collision module
 *
 * Decides when two bodies are collided.
 */

#include "dynamics.h"

/**
 * Checks whether two bodies collide, and sets the colliding flag
 * in them non-zero.
 *
 * @param [in,out] a,b The two bodies to check.
 * @param [in,out] nearestPoints The two closest points on the two primitive.
 * @param [in,out] lastSimplex ...
 *
 * @retval Nonzero if the two bodies intersect.
 * @retval Zero if the two bodies does not intersect.
 */
char COL_collide(DYN_Body *a, DYN_Body *b, double *nearestPoints, double *lastSimplex);

/**
 * Gets the penetration vector of two intersecting bodies. It uses the EPA
 * algorithm.
 *
 * @param [in,out] a,b Two intersecting bodies.
 * @param [in] startingSimplex starting simplex for the EPA algorithm.
 *      use COL_collide to get one suck simplex.
 * @param [out] penetrationVector The penetration vector (user provided array of 3 doubles.)
 */
void COL_getPenetrationVector(
    DYN_Body *a,
    DYN_Body *b,
    const double *startingSimplex,
    double *penetrationVector
);

