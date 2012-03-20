#include "dynamics.h"

/**
 * Checks whether two bodies collide, and sets the colliding flag
 * in them non-zero.
 *
 * @param [in,out] a,b The two bodies to check.
 *
 * @retval Nonzero if the two bodies intersect.
 * @retval Zero if the two bodies does not intersect.
 */
char COL_collide(DYN_Body *a, DYN_Body *b);
