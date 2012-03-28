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
 * @param [out] k, l Two interpolating factors, interpolating between the 3 vertices of the triangle.
 *
 * @retval Nonzero if the support vector is valid (the support point is on the triangle)
 * @retval Zero if no prependicular support vector exist.
 */
int getSupportVectorOfTriangle(const double *triangle, double *supportVector, double *k, double *l)
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
    *k = coefficients[0];
    *l = coefficients[1];
    return 1;
}

/**
 * Gets the support vector (the vector from the nearest point to the origin)
 *
 * @param [in] endPoints the two endpoint of the line segment.
 * @param [out] supportVector The support vector. It must be big enough to store 3 elements.
 * @param [out] factor Interpolation coefficient between the points.
 *
 * @retval Nonzero if the support vector is valid (the support point is on the linesegment)
 * @retval Zero if no prependicular support vector exist.
 */
int getSupportVectorOfLineSegment(const double *endPoints, double *supportVector, double *factor)
{
    double dirVector[3];
    double dirSquare;
    double k;

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
    *factor = k;
    k = -ALG_dotProduct(endPoints, dirVector) / dirSquare;
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
 * @param [in, out] remainingIndices An array of vertexCount elements whose elements indicate
 *      whether the vertex in the original array is kept or deleted.
 * @param [in,out] interpolatingFactors 2-element array, storing 0, 1 or 2 interpolating factors
 *      depending on the supporting simplex.
 *
 * @retval Nonzero if the simplex contains the origin.
 * @retval Zero otherwise.
 */
int getSupportVectorOfSimplex(
    double *simplexVertices,
    int *vertexCount,
    double *supportVector,
    char *remainingIndices,
    double *interpolatingFactors
)
{
    const double ORIGIN[3] = {0};
    double remainingVertices[12];
    double currentSimplex[12];
    double supportVectorTmp[3];
    double minLength = 0;
    char found = 0;
    int i, j, k, l;
    int newVertexCount;
    double factorsTmp[2]; //

    assert(*vertexCount <= 4);
    assert(*vertexCount >= 1);

    for (i = 0; i < *vertexCount; i++)
    {
        // 1 simplex
        memcpy(&currentSimplex[0], &simplexVertices[3*i], sizeof(double) * 3);
        if (!found || (ALG_dotProduct(currentSimplex, currentSimplex) < minLength) )
        {
            found = 1;
            memcpy(remainingVertices, currentSimplex, sizeof(double) * 3);
            newVertexCount = 1;
            minLength = ALG_dotProduct(currentSimplex, currentSimplex);
            memcpy(supportVector, currentSimplex, sizeof(supportVectorTmp));
            ALG_scale(supportVector, -1);
            memset(remainingIndices, 0, sizeof(char) * *vertexCount);
            remainingIndices[i] = 1;
        }
        for (j = i + 1; j < *vertexCount; j++)
        {
            // 2 simplex
            memcpy(&currentSimplex[3], &simplexVertices[3*j], sizeof(double) * 3);
            if (getSupportVectorOfLineSegment(currentSimplex, supportVectorTmp, factorsTmp))
            {
                if (ALG_dotProduct(supportVectorTmp, supportVectorTmp) < minLength)
                {
                    memcpy(remainingVertices, currentSimplex, sizeof(double) * 6);
                    newVertexCount = 2;
                    minLength = ALG_dotProduct(supportVectorTmp, supportVectorTmp);
                    memcpy(supportVector, supportVectorTmp, sizeof(supportVectorTmp));
                    memset(remainingIndices, 0, sizeof(char) * *vertexCount);
                    remainingIndices[i] = 1;
                    remainingIndices[j] = 1;
                    memcpy(interpolatingFactors, factorsTmp, sizeof(factorsTmp));
                }
            }
            for (k = j + 1; k < *vertexCount; k++)
            {
                // 3 simplex
                memcpy(&currentSimplex[6], &simplexVertices[3*k], sizeof(double) * 3);
                if (getSupportVectorOfTriangle(
                    currentSimplex,
                    supportVectorTmp,
                    &factorsTmp[0],
                    &factorsTmp[1])
                )
                {
                    if (ALG_dotProduct(supportVectorTmp, supportVectorTmp) < minLength)
                    {
                        memcpy(remainingVertices, currentSimplex, sizeof(double) * 9);
                        newVertexCount = 3;
                        minLength = ALG_dotProduct(supportVectorTmp, supportVectorTmp);
                        memcpy(supportVector, supportVectorTmp, sizeof(supportVectorTmp));
                        memset(remainingIndices, 0, sizeof(char) * *vertexCount);
                        remainingIndices[i] = 1;
                        remainingIndices[j] = 1;
                        remainingIndices[k] = 1;
                        memcpy(interpolatingFactors, factorsTmp, sizeof(factorsTmp));
                    }
                }
                for (l = k + 1; l < *vertexCount; l++)
                {
                    // 4 simplex
                    if (isPointInTetraHedron(ORIGIN, simplexVertices))
                    {
                            memset(remainingIndices, 1, sizeof(char) * *vertexCount);
                        return 1;
                    }
                }
            }
        }
    }
    memcpy(simplexVertices, remainingVertices, sizeof(double) * newVertexCount * 3);
    *vertexCount = newVertexCount;

    return 0;
}

/**
 * Calculates if two convex bodies intersect. (See GJK algorithm)
 *
 * @param [in] bodyAVertices, bodyBVertices The vertices of the two body, in arbitrary order.
 * @param [in] bodyAVertexCount, bodyBVertexCount The vertex count of the two body.
 * @param [in,out] nearestPoints 6 element array, stores the two nearest points of the
 *      two bodies.
 */
int isConvexBodiesIntersect(
    const double *bodyAVertices,
    int bodyAVertexCount,
    const double *bodyBVertices,
    int bodyBVertexCount,
    double *nearestPoints
)
{
    double simplexVertices[12];
    char simplexVertexIndexPairs[4][2]; // Stores the pairs of indices of the vertices the simplex vertices is calculated from.
    int simplexVertexCount = 0;
    double supportVector[3];
    char alreadyInspected[bodyAVertexCount][bodyBVertexCount];
    double interpolationFactors[2];

    assert(bodyAVertexCount > 0);
    assert(bodyBVertexCount > 0);

    memset(alreadyInspected, 0, bodyAVertexCount * bodyBVertexCount);
    // Set starting state.
    ALG_getPointToPointVector(simplexVertices, bodyBVertices, bodyAVertices);
    alreadyInspected[0][0] = 1;
    simplexVertexCount = 1;
    simplexVertexIndexPairs[0][0] = 0;
    simplexVertexIndexPairs[0][1] = 0;
    // Find nearest point.
    for(;;)
    {
        char remainingVertices[4]; //
        if (getSupportVectorOfSimplex(
            simplexVertices,
            &simplexVertexCount,
            supportVector,
            remainingVertices,
            interpolationFactors)
        )
        {
            // simplex was a tetrahedron an origin was in the tetrahedron. This means intersection.
            return 1;
        }
        {
            // Update the vertex index pairs based on the remaining vertices.
            char tmp[4][2];
            int i;
            int c = 0;
            for (i = 0; i < 4; i++)
            {
                if (remainingVertices[i])
                {
                    memcpy(tmp[c], simplexVertexIndexPairs[i], sizeof(tmp[c]));
                    c++;
                }
            }
            memcpy(simplexVertexIndexPairs, tmp, sizeof(tmp));
            // Now simplexVertices array and the simplexVertexIndexPairs array is in sync.
        }
        // Find the farthest point of the Minkowski difference in the supportVector's direction.
        {
            double maxProduct;
            double minProduct;
            int i;
            double maxVertex[3]; // The farthest point of the A body.
            double minVertex[3];  // The nearest point of the B body.
            int maxIndex; // index of the maximum vertex
            int minIndex; // index of the minimum vertex
            double minkowskiDifference[3];
            double svLength = ALG_dotProduct(supportVector, supportVector);

            if (svLength < 1e-9)
            {
                // support vector is null vector, it's sure that the minkowski difference touches the
                // origin, intersection assured.
                return 1;
            }

            for (i = 0; i < bodyAVertexCount; i++)
            {
                double currentProduct = ALG_dotProduct(supportVector, &bodyAVertices[3*i]);
                if (!i || (currentProduct > maxProduct))
                {
                    maxProduct = currentProduct;
                    maxIndex = i;
                }
            }
            memcpy(maxVertex, &bodyAVertices[3*maxIndex], sizeof(double) * 3);

            for (i = 0; i < bodyBVertexCount; i++)
            {
                double currentProduct = ALG_dotProduct(supportVector, &bodyBVertices[3*i]);
                if (!i || (currentProduct < minProduct))
                {
                    minProduct = currentProduct;
                    minIndex = i;
                }
            }
            memcpy(minVertex, &bodyBVertices[3*minIndex], sizeof(double) * 3);
            if (alreadyInspected[maxIndex][minIndex])
            {
                // We would add a vertex that's already inspected.
                // This mean the two bodies does not intersect.
                // Calculate the two nearest points.
                int i;
                double refPoint[3];
                double pointA[3];
                memcpy(
                    refPoint,
                    &bodyAVertices[3 * simplexVertexIndexPairs[0][0]],
                    sizeof(refPoint)
                );
                memcpy(pointA, refPoint, sizeof(pointA));
                for (i = 1; i < simplexVertexCount; i++)
                {
                    double sideVector[3];
                    ALG_getPointToPointVector(
                        sideVector,
                        refPoint,
                        &bodyAVertices[3 * simplexVertexIndexPairs[i][0]]
                    );
                    ALG_scale(sideVector, interpolationFactors[i - 1]);
                    ALG_translate(pointA, sideVector);
                }
                memcpy(&nearestPoints[0], pointA, 3 * sizeof(*nearestPoints));
                memcpy(&nearestPoints[3], pointA, 3 * sizeof(*nearestPoints));
                ALG_translate(&nearestPoints[3], supportVector);
                return 0;
            }
            alreadyInspected[maxIndex][minIndex] = 1;
            assert(simplexVertexCount < 4);
            ALG_getPointToPointVector(minkowskiDifference, minVertex, maxVertex);
            memcpy(&simplexVertices[3*simplexVertexCount], minkowskiDifference, sizeof(double) * 3);
            simplexVertexIndexPairs[simplexVertexCount][0] = maxIndex;
            simplexVertexIndexPairs[simplexVertexCount][1] = minIndex;
            simplexVertexCount++;
        }
    }
    assert(0);
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
 * @param [out] vertexCount The count of vertices of the body.
 */
void getBodyVertices(const DYN_Body *a, double *vertices, int *vertexCount)
{
    const double *orientation = a->orientation;
    const double *position = a->position;
    int i;

    DYN_BodyStaticAttributes *attr = a->staticAttributes;
    switch (attr->shape)
    {
        case DYN_BS_CUBOID:
            getCuboidRawVertices(attr, vertices);
            *vertexCount = 8;
        break;
        default:
            assert(0);
    }
    for (i = 0; i < *vertexCount; i++)
    {
        double tmp[3];
        ALG_transform(tmp, &vertices[3*i], orientation);
        memcpy(&vertices[3*i], tmp, sizeof(tmp));
        ALG_translate(&vertices[3*i], position);
    }
}

double COL_latestNearestPoints[6] = {0};

double *COL_queryLatestNearest()
{
    return COL_latestNearestPoints;
}

char COL_collide(DYN_Body *a, DYN_Body *b)
{
    double verticesA[24];
    double verticesB[24];
    int vertexCountA, vertexCountB;

    getBodyVertices(a, verticesA, &vertexCountA);
    getBodyVertices(b, verticesB, &vertexCountB);

    if (isConvexBodiesIntersect(
        verticesA,
        vertexCountA,
        verticesB,
        vertexCountB,
        COL_latestNearestPoints)
    )
    {
        a->colliding = 1;
        b->colliding = 1;
        return 1;
    }
    return 0;
}
