#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "collision.h"
#include "dynamics.h"
#include "algebra.h"

/**
 * Determines if point is inside a tetrahedron
 *
 * @param [in] point The point [x,y,z]
 * @param [in] tetrahedron The vertices of the tetrahedron (12 elements)
 *
 * @retval Nonzero if the point is inside the tetrahedron
 * @retval Zero otherwise.
 */
int isPointInTetraHedron(const double *point,const double *tetrahedron)
{
    double matrix[9]; //< A matrix of the 3 column vectors from the first vertex to the other three.
    int i, j;
    double relLocation[3]; //< Relative location of the point from the tetrahedron's first vertex.
    double coefficients[3]; //< The factors on the vectors to get the given point.
    double sumCoeff; //< sum of the 3 coefficients;

    for (i = 0, j = 0; i < 9; i++, j++)
    {
        if (j == 3) j = 0; //< j = i % 3
        matrix[i] = tetrahedron[i + 3] - tetrahedron[j];
    }
    // calculate the relative location of the point
    for (i = 0; i < 3; i++)
    {
        relLocation[i] = point[i] - tetrahedron[i];
    }
    // The matrix currently stores the row vector, convert it to store column vectors.
    ALG_transposeMatrix(matrix);
    // Solve the equations of the coefficients.
    if (!ALG_solveSystem3(coefficients, matrix, relLocation))
    {
        // Tetrahedron was actually a triangle. (all 4 vertices lie in the same plane).
        return 0;
    }
    sumCoeff = 0;
    for (i = 0; i < 3; i++)
    {
        // Negative coefficients assure that the point is outside.
        if (coefficients[i] < 0) return 0;
        sumCoeff += coefficients[i];
    }
    if (sumCoeff > 1)
    {
        // In this case the point is outside the plane opposite the first vertex.
        return 0;
    }
    return 1;
}

/**
 * Gets the support vector (the vector from the nearest point to the origin)
 *
 * @param [in] triangle the vertices of the triangle (9 elements [x,y,z,x,y,z,x,y,z])
 * @param [out] supportVector The support vector. It must be big enough to store 3 elements.
 *
 * @retval Nonzero if the support vector is valid (the support point is on the triangle)
 * @retval Zero if no prependicular support vector exist.
 */
int getSupportVectorOfTriangle(const double *triangle, double *supportVector)
{
     /* 3 column vectors. Two side vectors of the triangle for the first vertex
      * and one which is prependicular to them.
      */
    double matrix[9];
    int i, j;
    double coefficients[3]; //< Solution of the Mx = b. Where M is the matrix, -b is the first vertex of the triangle.
    double rightSide[3]; //< right side of the vector equation.

    // calculate the side vectors
    for (i = 0, j = 0; i < 6; i++, j++)
    {
        if (j == 3) j = 0; //< j = i % 3;
        matrix[i] = triangle[i + 3] - triangle[j];
    }
    // calculate the prependicular vector.
    ALG_crossProduct(&matrix[6], &matrix[0], &matrix[3]);
    memcpy(supportVector, &matrix[6], sizeof(*supportVector) * 3);
    // Turn the row vectors into column vectors.
    ALG_transposeMatrix(matrix);
    // Calculate the right side.
    for (i = 0; i < 3; i++)
    {
        rightSide[i] = -triangle[i];
    }
    // Solve the coefficients.
    if (!ALG_solveSystem3(coefficients, matrix, rightSide))
    {
        // The triangle is actually a line segment. No support vector exists.
        return 0;
    }
    // Check coefficients
    if (
        (coefficients[0] < 0) ||
        (coefficients[1] < 0) ||
        (coefficients[0] + coefficients[1] >= 1)
    )
    {
        // support point is outside the triangle no vector exist.
        return 0;
    }
    // calculate the support vector
    ALG_scale(supportVector, coefficients[2]);
    return 1;
}

/**
 * Gets the support vector (the vector from the nearest point to the origin)
 *
 * @param [in] endPoints the two endpoint of the line segment.
 * @param [out] supportVector The support vector. It must be big enough to store 3 elements.
 *
 * @retval Nonzero if the support vector is valid (the support point is on the linesegment)
 * @retval Zero if no prependicular support vector exist.
 */
int getSupportVectorOfLineSegment(const double *endPoints, double *supportVector)
{
    double dirVector[3];
    double k; //< interpolating coefficient.
    double dirSquare;

    ALG_getPointToPointVector(dirVector, endPoints, &endPoints[3]);
    dirSquare = ALG_dotProduct(dirVector, dirVector);
    if (dirSquare == 0)
    {
        // The line seqment is just a point, no support point exists.
        return 0;
    }

    k = -ALG_dotProduct(endPoints, dirVector) / dirSquare;
    if ((k < 0) || (k > 1))
    {
        // support point is outside the segment.
        return 0;
    }
    memcpy(supportVector, endPoints, sizeof(*supportVector) * 3); // A
    ALG_scale(dirVector, k); // calculate kv
    ALG_translate(supportVector, dirVector); // calculate A + kv
    ALG_scale(supportVector, -1); // negate it.
    return 1;

}

/**
 * Gets the support vector of a simplex.
 * (The vector from the closest point to the origin to the origin.)
 *
 * @param [in,out] simplexVertices Vertices of the convex simplex. After the
 *      calculation several vertices may be removed from the array which does not
 *      contribute to the support point (see GJK algorithm)
 * @param [in,out] vertexCount Count of the vertices in the simplex. This value may change.
 * @param [in,out] supportVector The support vector will be stored here.
 *      (The user must provide an array to store 3 elements.)
 *
 * @retval Nonzero if the simplex contains the origin.
 * @retval Zero otherwise.
 */
int getSupportVectorOfSymplex(double *simplexVertices, double *vertexCount, double *supportVector)
{
    assert(*vertexCount <= 4);
    assert(*vertexCount >= 1);
    return 0;
}

/**
 * Calculates if two convex bodies intersect. (See GJK algorithm)
 *
 * @param [in] bodyAVertices, bodyBVertices The vertices of the two body, in arbitrary order.
 */
int isConvexBodiesIntersect(
    const double *bodyAVertices,
    int bodyAVertexCount,
    const double *bodyBVertices,
    int bodyBVertexCount
)
{
    return 0;
}

/**
 * Gets the vertices of a cuboid in body space.
 *
 * @param [in] attr Static attributes of the cuboid.
 * @param [out] vertices The raw vertices packed in [x,y,z,x,y,z,...] format.
 *      The array provided must be big enough to hold the data.
 */
void getCuboidRawVertices(const DYN_BodyStaticAttributes *attr, double *vertices)
{
    int i = 0;
    assert(attr->shape == DYN_BS_CUBOID);
    // dimensions distances from the mass center.
    double w = attr->cuboidAttributes.width * 0.5;
    double h = attr->cuboidAttributes.height * 0.5;
    double d = attr->cuboidAttributes.depth * 0.5;
    // Fill the vertices array.
    vertices[i++] = w; vertices[i++] = h; vertices[i++] = d;
    vertices[i++] = w; vertices[i++] = h; vertices[i++] = -d;
    vertices[i++] = w; vertices[i++] = -h; vertices[i++] = d;
    vertices[i++] = w; vertices[i++] = -h; vertices[i++] = -d;
    vertices[i++] = -w; vertices[i++] = h; vertices[i++] = d;
    vertices[i++] = -w; vertices[i++] = h; vertices[i++] = -d;
    vertices[i++] = -w; vertices[i++] = -h; vertices[i++] = d;
    vertices[i++] = -w; vertices[i++] = -h; vertices[i++] = -d;
}

/**
 * Queries the current vertex locations of a body.
 *
 * @param [in] a The body.
 * @param [in,out] The array of vertices. Packed in [x,y,z,x,y,z,...] format.
 *      The array provided must be big enough to hold the data.
 */
void getBodyVertices(const DYN_Body *a, double *vertices)
{
    const double *orientation = a->orientation;
    const double *position = a->position;
    int vertexCount;
    int i;

    DYN_BodyStaticAttributes *attr = a->staticAttributes;
    switch (attr->shape)
    {
        case DYN_BS_CUBOID:
            getCuboidRawVertices(attr, vertices);
            vertexCount = 8;
        break;
        default:
            assert(0);
    }
    for (i = 0; i < vertexCount; i++)
    {
        double tmp[3];
        ALG_transform(tmp, &vertices[3*i], orientation);
        memcpy(&vertices[3*i], tmp, sizeof(tmp));
        ALG_translate(&vertices[3*i], position);
    }
}

void COL_collide(DYN_Body *a, DYN_Body *b)
{
    double verticesA[24];
    double verticesB[24];

    getBodyVertices(a, verticesA);
    getBodyVertices(b, verticesB);


}
