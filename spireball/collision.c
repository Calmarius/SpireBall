#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "collision.h"
#include "dynamics.h"
#include "algebra.h"
#include "avltree.h"
#include "dynarray.h"

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
    if (k) *k = coefficients[0];
    if (l) *l = coefficients[1];
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
 * Gets the vertex which is the farthest (or nearest) in the given direction.
 *
 * @param [in] vertices Vertices (series of 3 doubles per vertex)
 * @param [in] vertexCount Count of vertices
 * @param [in] direction The direction of the query.
 * @param [in] nearest When it's nonzero, it will find the nearest in the given
 *      direction instead of the farthest.
 * @param [out] vertexIndex The index of the vertex which is the farthest (or nearest) .
 *      It can be NULL, when this info is not needed.
 * @param [in,out] vertex The coordinates of the vertex which is the farthest (or nearest) .
 *      It can be NULL, when this info is not needed.
 */
void getExtremePointOfVertices(
    const double *vertices,
    int vertexCount,
    const double *direction,
    char nearest,
    int *vertexIndex,
    double *vertex
)
{
    int maxIndex;
    int i;
    double maxProduct;

    if (nearest)
    {
        for (i = 0; i < vertexCount; i++)
        {
            double currentProduct = ALG_dotProduct(direction, &vertices[3*i]);
            if (!i || (currentProduct < maxProduct))
            {
                maxProduct = currentProduct;
                maxIndex = i;
            }
        }
    }
    else
    {
        for (i = 0; i < vertexCount; i++)
        {
            double currentProduct = ALG_dotProduct(direction, &vertices[3*i]);
            if (!i || (currentProduct > maxProduct))
            {
                maxProduct = currentProduct;
                maxIndex = i;
            }
        }
    }
    if (vertex)
    {
        memcpy(vertex, &vertices[3*maxIndex], sizeof(double) * 3);
    }
    if (vertexIndex) *vertexIndex = maxIndex;
}

/**
 * Gets the extreme vertex of the Minkowski difference of the two vertex
 * sets in a direction
 *
 * @param [in] verticesA, vertexACount Vertices of A body.
 * @param [in] verticesB, vertexBCount Vertices of the B body.
 * @param [in] direction The direction.
 * @param [out] indexA, indexB The index of the chosen vertex of the two bodies.
 *      These parameters can be NULL.
 * @param [out] vertex The vertex on the Minkowski difference.
 */
void getExtremePointOfMinkowskiDifference(
    const double *verticesA,
    int vertexACount,
    const double *verticesB,
    int vertexBCount,
    const double *direction,
    int *indexA,
    int *indexB,
    double *vertex
)
{
    double maxVertex[3];
    double minVertex[3];

    getExtremePointOfVertices(
        verticesA,
        vertexACount,
        direction,
        0,
        indexA,
        maxVertex
    );
    getExtremePointOfVertices(
        verticesB,
        vertexBCount,
        direction,
        1,
        indexB,
        minVertex
    );
    ALG_getPointToPointVector(vertex, minVertex, maxVertex);

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
    double *nearestPoints,
    double *lastSimplex
)
{
    double simplexVertices[12];
    char simplexVertexIndexPairs[4][2]; // Stores the pairs of indices of the vertices the simplex vertices is calculated from.
    int simplexVertexCount = 0;
    double supportVector[3] = {0};
    double prevSupportVector[3] = {0};
    double interpolationFactors[2];

    assert(bodyAVertexCount > 0);
    assert(bodyBVertexCount > 0);

    // Set starting state.
    ALG_getPointToPointVector(simplexVertices, bodyBVertices, bodyAVertices);
    simplexVertexCount = 1;
    simplexVertexIndexPairs[0][0] = 0;
    simplexVertexIndexPairs[0][1] = 0;
    // Find nearest point.
    for(;;)
    {
        char remainingVertices[4]; //
        memcpy(prevSupportVector, supportVector, sizeof(prevSupportVector));
        if (getSupportVectorOfSimplex(
            simplexVertices,
            &simplexVertexCount,
            supportVector,
            remainingVertices,
            interpolationFactors)
        )
        {
            // simplex was a tetrahedron an origin was in the tetrahedron. This means intersection.
            if (lastSimplex)
            {
                memcpy(lastSimplex, simplexVertices, 12 * sizeof(*lastSimplex));
            }
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
            int maxIndex; // index of the maximum vertex
            int minIndex; // index of the minimum vertex
            double minkowskiDifference[3];
            double svLength = ALG_dotProduct(supportVector, supportVector);
            if (svLength < 1e-9)
            {
                // support vector is zero, intersection or touching is ensured.
                // This is a corner case.
                if (lastSimplex)
                {
                    int i;
                    memcpy(lastSimplex, simplexVertices, 12 * sizeof(*lastSimplex));
                    // Add some random edge vertices of the difference to the last simplex if needed.
                    for (i = simplexVertexCount; i < 4; i++)
                    {
                        for(;;)
                        {
                            int j = 3;
                            double randomSupport[3];
                            // get random support vector
                            while (j--)
                            {
                                randomSupport[j] = (rand() % 255 - 128) / 255.0;
                            }
                            // Get extreme vector
                            getExtremePointOfMinkowskiDifference(
                                bodyAVertices,
                                bodyAVertexCount,
                                bodyBVertices,
                                bodyBVertexCount,
                                randomSupport,
                                &maxIndex,
                                &minIndex,
                                minkowskiDifference
                            );
                            // Check if the new vertex already exist
                            for (j = 0; j < i; j++)
                            {
                                double tmp[3];
                                ALG_getPointToPointVector(
                                    tmp,
                                    &lastSimplex[3*j],
                                    minkowskiDifference
                                );
                                if (ALG_dotProduct(tmp, tmp) < 1e-9)
                                {
                                    // This vertex already exists, try again.
                                    goto try_again;
                                }
                            }
                            // Check if the new simplex's area or volume is non zero
                            {
                                double sideVectors[9];
                                int j;
                                for (j = 0; j < i; j++)
                                {
                                    ALG_getPointToPointVector(
                                        &sideVectors[3*j],
                                        minkowskiDifference,
                                        &simplexVertices[3*j]
                                    );
                                }
                                if (i == 2)
                                {

                                    // Current simplex is a 2-simplex (triangle)
                                    double det[3];
                                    ALG_crossProduct(det, &sideVectors[0], &sideVectors[3]);
                                    printf("2-simplex determinant is: %g\n", ALG_dotProduct(det, det));
                                    if (ALG_dotProduct(det, det) < 1e-9)
                                    {
                                        // This would be a triangle with zero area, bad vertex.
                                        continue;
                                    }
                                }
                                else if (i == 3)
                                {
                                    // Current simplex is a 3-simplex (tetrahedron)
                                    double det = ALG_getDeterminant(sideVectors);
                                    printf("3-simplex determinant is: %g\n", det);
                                    if (fabs(det) < 1e-9)
                                    {
                                        // This would be a tetrahedron with zero volume.
                                        continue;
                                    }
                                }
                                else
                                {
                                    assert(0);
                                }

                            }
                            // Vertex is new so add it.
                            break;
                        try_again:;
                        }
                        memcpy(
                            &lastSimplex[3*i],
                            minkowskiDifference,
                            sizeof(minkowskiDifference)
                        );
                    }
                }
                return 1;
            }

            // Calculate the farthest point of the A body.
            getExtremePointOfMinkowskiDifference(
                bodyAVertices,
                bodyAVertexCount,
                bodyBVertices,
                bodyBVertexCount,
                supportVector,
                &maxIndex,
                &minIndex,
                minkowskiDifference
            );
            {
                double difference[3];
                ALG_getPointToPointVector(difference, prevSupportVector, supportVector);
                if (ALG_dotProduct(difference, difference) < 1e-6)
                {
                    // We would add a vertex that's already inspected.
                    // This maymean the two bodies does not intersect.
                    // Calculate the two nearest points.
                    if (nearestPoints)
                    {
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
                    }
                    return 0;
                }
            }
            assert(simplexVertexCount < 4);
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

typedef struct
{
    double vertices[9];
    double supportVector[3];
    double squaredDistance;
} PolytopeSide;

static int polytopeSideComparer(const void *a, const void *b)
{
    PolytopeSide *aside = (PolytopeSide*)a;
    PolytopeSide *bside = (PolytopeSide*)b;
    int i;

    if (aside->squaredDistance < bside->squaredDistance) return -1;
    if (aside->squaredDistance > bside->squaredDistance) return 1;
    // if distance is equal order by vertices. It's a corner case but may happen
    for (i = 0; i < 9; i++)
    {
        double tmp = aside->vertices[i] - bside->vertices[i];
        if (fabs(tmp) > 1e-9)
        {
            return tmp;
        }
    }

    return 0;
}

static char isSupportOnEdge(int k, int l)
{
    if (fabs(k) < 1e-6) return 1;
    if (fabs(l) < 1e-6) return 1;
    if (fabs(k + l - 1) < 1e-6) return 1;
    return 0;
}

void psideDump(const void *key, const void *value)
{
    PolytopeSide *pside = (PolytopeSide*)key;

    printf("polyside in tree ([%g, %g, %g], [%g, %g, %g], [%g, %g, %g]), d*d: %g, [%g, %g, %g]\n",
        pside->vertices[0], pside->vertices[1], pside->vertices[2],
        pside->vertices[3], pside->vertices[4], pside->vertices[5],
        pside->vertices[6], pside->vertices[7], pside->vertices[8],
        pside->squaredDistance,
        pside->supportVector[0], pside->supportVector[1], pside->supportVector[2]
    );

}

/**
 * Performs EPA algorithm.
 *
 * @note The two bodies must intersect.
 *
 * @param [in] bodyAVertices, bodyAVertexCount The vertices of the body A.
 * @param [in] bodyBVertices, bodyBVertexCount The vertices of the body B.
 * @param [in] startingSimplex The vertices of the starting tetrahedron [x, y, z, x, y, z, ...].
 *      Origin must be inside.
 * @param [in,out] penetrationVector The penetration vector, user provided array of 3 doubles.
 *
 */
void getPenetrationVector(
    const double *bodyAVertices,
    int bodyAVertexCount,
    const double *bodyBVertices,
    int bodyBVertexCount,
    const double *startingSimplex,
    double *penetrationVector
)
{
    double nullVector[3] = {0, 0, 0};
    AVL_Tree treeIndex;
    DYNA_Array dynArray;
    double sumVertex[3] = {0, 0, 0}; //< All politope vertices summed together;
    int numberOfVertices = 0;
    double centerPoint[3] = {0, 0, 0};
    int i, j, k;
    enum
    {
        NOTHING_SPECIAL,
        EDGE_AT_ORIGIN,
        FACE_AT_ORIGIN
    } specialCase = NOTHING_SPECIAL;
    double supportVector[3];

    AVL_initialize(&treeIndex, polytopeSideComparer);
    DYNA_initialize(&dynArray, sizeof(PolytopeSide*));
    // Do corner case checking here.
    {
        double tmp[9];
        double originSegment[6];
        double originFace[9];
        int offendingEdgeCount = 0;
        int offendingFaceCount = 0;
        // Check any vertices are near the origin
        for (i = 0; i < 4; i++)
        {
            if (ALG_dotProduct(&startingSimplex[3 * i], &startingSimplex[3 * i]) < 1e-9)
            {
                // One vertex is at the origin, penetration vector is zero.
                printf("Vertex is at the origin.\n");
                memcpy(penetrationVector, nullVector, sizeof(nullVector));
                return;
            }
        }
        // Check if any of the edges intersect the origin.
        for (i = 0; i < 4; i++)
        {
            for (j = i + 1; j < 4; j++)
            {
                memcpy(tmp, &startingSimplex[3 * i], 3 * sizeof(double));
                memcpy(&tmp[3], &startingSimplex[3 * j], 3 * sizeof(double));
                if (getSupportVectorOfLineSegment(tmp, supportVector, 0))
                {
                    if (ALG_dotProduct(supportVector, supportVector) < 1e-9)
                    {
                        printf("Edge contains the origin.\n");
                        memcpy(originSegment, tmp, sizeof(originSegment));
                        offendingEdgeCount++;
                    }
                }
            }
        }
        if (offendingEdgeCount == 1)
        {
            specialCase = EDGE_AT_ORIGIN;
            goto cornerCaseTestingFinished;
        }
        else
        {
            printf("Multiple edges are contain the origin. The initial tetrahedron is a triangle.\n");
            memcpy(penetrationVector, nullVector, sizeof(nullVector));
            return;
        }
        // Check if any of the face intersect the origin.
        for (i = 0; i < 4; i++)
        {
            for (j = i + 1; j < 4; j++)
            {
                for (k = j + 1; k < 4; k++)
                {
                    memcpy(tmp, &startingSimplex[3 * i], 3 * sizeof(double));
                    memcpy(&tmp[3], &startingSimplex[3 * j], 3 * sizeof(double));
                    memcpy(&tmp[6], &startingSimplex[3 * k], 3 * sizeof(double));
                    if (getSupportVectorOfTriangle(tmp, supportVector, 0, 0))
                    {
                        if (ALG_dotProduct(supportVector, supportVector) < 1e-9)
                        {
                            printf("Face is at origin.\n");
                            memcpy(originFace, tmp, sizeof(originFace));
                            offendingFaceCount++;
                        }
                    }
                }
            }
        }
        if (offendingFaceCount == 1)
        {
            specialCase = FACE_AT_ORIGIN;
        }
        else
        {
            printf("Multiple edges are contain the origin. The initial tetrahedron is a triangle.\n");
            memcpy(penetrationVector, nullVector, sizeof(nullVector));
            return;
        }
    }
cornerCaseTestingFinished:
    // Add the initial triangles
    {
        double extraVertex[3];

        if (specialCase != NOTHING_SPECIAL)
        {
            // On special case, we need to calculate the center point of the simplex.
            // Which is the average of the vertices.
            for (i = 0; i < 4; i++)
            {
                ALG_translate(centerPoint, &startingSimplex[3 * i]);
            }
            ALG_scale(centerPoint, 0.25); //< /4
            memcpy(supportVector, centerPoint, sizeof(supportVector));
            ALG_scale(supportVector, -1);
            getExtremePointOfMinkowskiDifference(
                bodyAVertices,
                bodyAVertexCount,
                bodyBVertices,
                bodyBVertexCount,
                supportVector,
                0,
                0,
                extraVertex
            );
        }
    }

    {
        // 4 triangles
        numberOfVertices = 4;
        printf("*******************\n");
        printf("Politope vertices:\n");
        for (i = 0; i < 4; i++)
        {
            int k = 0;
            double l, m;
            PolytopeSide *tmp = malloc(sizeof(PolytopeSide));

            printf("[%g, %g, %g]\n", startingSimplex[i * 3], startingSimplex[i * 3 + 1], startingSimplex[i * 3 + 2]);
            ALG_translate(sumVertex, &startingSimplex[i * 3]); //< Sum the vertives together;
            // Get a triange by taking the vertices of the tetraheadron, except the i-th.
            for (j = 0; j < 4; j++)
            {
                if (i == j) continue;
                memcpy(
                    &tmp->vertices[k * 3],
                    &startingSimplex[j * 3],
                    3 * sizeof(double)
                );
                k++;
            }
            if (
                getSupportVectorOfTriangle(tmp->vertices, tmp->supportVector, &l, &m) ||
                (isSupportOnEdge(l, m))
            )
            {
                PolytopeSide *pside;

                tmp->squaredDistance = ALG_dotProduct(
                    tmp->supportVector,
                    tmp->supportVector
                );
                DYNA_add(&dynArray, &tmp);
                AVL_add(&treeIndex, tmp, 0);
                pside = (PolytopeSide*)tmp;
                printf("Added pside ([%g, %g, %g], [%g, %g, %g], [%g, %g, %g]), d*d: %g, [%g, %g, %g]\n",
                    pside->vertices[0], pside->vertices[1], pside->vertices[2],
                    pside->vertices[3], pside->vertices[4], pside->vertices[5],
                    pside->vertices[6], pside->vertices[7], pside->vertices[8],
                    pside->squaredDistance,
                    pside->supportVector[0], pside->supportVector[1], pside->supportVector[2]
                );
            }
        }
        printf("*******************\n");
    }
    // Do EPA here.
    for(;;)
    {
        // Debug dump
        printf("*******************\n");
        AVL_dumpTree(&treeIndex, psideDump);
        // Get the face which is nearest to the origin.
        const AVL_Node *least = AVL_getLeast(&treeIndex);
        const PolytopeSide *pside;
        double newVertex[3];
        PolytopeSide *newSide = 0;
        int i, j;

        if (!least)
        {
            memset(penetrationVector, 0, 3 * sizeof(*penetrationVector));
            goto cleanup;
        }
        pside = AVL_getKey(least);
        printf("Chosen pside ([%g, %g, %g], [%g, %g, %g], [%g, %g, %g]), d*d: %g, [%g, %g, %g]\n",
            pside->vertices[0], pside->vertices[1], pside->vertices[2],
            pside->vertices[3], pside->vertices[4], pside->vertices[5],
            pside->vertices[6], pside->vertices[7], pside->vertices[8],
            pside->squaredDistance,
            pside->supportVector[0], pside->supportVector[1], pside->supportVector[2]
        );
        /*if (pside->squaredDistance < 1e-9)
        {
            double sideA[3];
            double sideB[3];
            double normal[3];
            double center[3];
            double centerDir[3];

            printf("Zero vector case.\n");
            // Calculate support vector if the distance is zero
            ALG_getPointToPointVector(sideA, &pside->vertices[0], &pside->vertices[3]);
            ALG_getPointToPointVector(sideB, &pside->vertices[0], &pside->vertices[6]);
            ALG_crossProduct(normal, sideA, sideB);
            printf("sideA [%g, %g, %g]\n", sideA[0], sideA[1], sideA[2]);
            printf("sideB [%g, %g, %g]\n", sideB[0], sideB[1], sideB[2]);
            // We have a normal, now determine if it's pointing to the right direction.
            // Get the center of the current politope.
            memcpy(center, sumVertex, sizeof(sumVertex));
            ALG_scale(center, 1.0/numberOfVertices);
            printf("Center point is: [%g, %g, %g]\n", center[0], center[1], center[2]);
            ALG_getPointToPointVector(centerDir, &pside->vertices[0], center);
            if (ALG_dotProduct(centerDir, normal) > 0)
            {
                // Normal pointing inside.
                // Reverse it.
                ALG_scale(normal, -1);
            }
            memcpy(pside->supportVector, normal, sizeof(normal));
            printf(
                "The new support vector is: [%g, %g, %g]\n",
                pside->supportVector[0],
                pside->supportVector[1],
                pside->supportVector[2]
            );
        }*/

        // Use its support vector to calculate and add a new vertex.
        getExtremePointOfMinkowskiDifference(
            bodyAVertices,
            bodyAVertexCount,
            bodyBVertices,
            bodyBVertexCount,
            pside->supportVector,
            0,
            0,
            newVertex
        );

        // Check if its near the current triangle's vertices
        for (i = 0; i < 3; i++)
        {
            double tmp[3];
            ALG_getPointToPointVector(tmp, newVertex, &pside->vertices[3 * i]);
            if (ALG_dotProduct(tmp, tmp) < 1e-6)
            {
                // Yes the vertex is nearby. We finished.
                memcpy(
                    penetrationVector,
                    pside->supportVector,
                    sizeof(pside->supportVector)
                );
                goto cleanup;
            }
        }
        // Add the new vertex to the sum
        ALG_translate(sumVertex, newVertex);
        numberOfVertices++;
        printf(
            "The new vertex is: [%g, %g, %g]\n",
            newVertex[0],
            newVertex[1],
            newVertex[2]
        );

        // Add new triangles to the index.
        // 3 new triangles.
        for (i = 0; i < 3; i++)
        {
            int k = 0;
            double l, m;

            newSide = malloc(sizeof(PolytopeSide));
            // 2 vertex from the old triangle
            for (j = 0; j < 3; j++)
            {
                if (i == j) continue;
                memcpy(&newSide->vertices[3 * k], &pside->vertices[3 * j], 3 * sizeof(double));
                k++;
            }
            // plus the new vertex
            memcpy(&newSide->vertices[6], newVertex, 3 * sizeof(double));
            // Calculate support vector on these triangles
            if (getSupportVectorOfTriangle(newSide->vertices, newSide->supportVector, &l, &m) || isSupportOnEdge(l, m))
            {
                PolytopeSide *pside;
                newSide->squaredDistance = ALG_dotProduct(
                    newSide->supportVector,
                    newSide->supportVector
                );
                DYNA_add(&dynArray, &newSide);
                AVL_add(&treeIndex, newSide, 0);
                pside = (PolytopeSide*)newSide;
                printf("Added pside ([%g, %g, %g], [%g, %g, %g], [%g, %g, %g]), d*d: %g, [%g, %g, %g]\n",
                    pside->vertices[0], pside->vertices[1], pside->vertices[2],
                    pside->vertices[3], pside->vertices[4], pside->vertices[5],
                    pside->vertices[6], pside->vertices[7], pside->vertices[8],
                    pside->squaredDistance,
                    pside->supportVector[0], pside->supportVector[1], pside->supportVector[2]
                );
            }
        }
        // Remove the checked triangle
        printf("Deleted pside ([%g, %g, %g], [%g, %g, %g], [%g, %g, %g]), d*d: %g, [%g, %g, %g]\n",
            pside->vertices[0], pside->vertices[1], pside->vertices[2],
            pside->vertices[3], pside->vertices[4], pside->vertices[5],
            pside->vertices[6], pside->vertices[7], pside->vertices[8],
            pside->squaredDistance,
            pside->supportVector[0], pside->supportVector[1], pside->supportVector[2]
        );
        AVL_delete(&treeIndex, pside);
    }
cleanup:
    AVL_deinitialize(&treeIndex);
    DYNA_deinitialize(&dynArray);
}

void COL_getPenetrationVector(
    DYN_Body *a,
    DYN_Body *b,
    const double *startingSimplex,
    double *penetrationVector
)
{
    double verticesA[24]; //< Not fool proof
    double verticesB[24];
    int vertexCountA, vertexCountB;

    getBodyVertices(a, verticesA, &vertexCountA);
    getBodyVertices(b, verticesB, &vertexCountB);

    getPenetrationVector(
        verticesA,
        vertexCountA,
        verticesB,
        vertexCountB,
        startingSimplex,
        penetrationVector
    );
}

char COL_collide(DYN_Body *a, DYN_Body *b, double *nearestPoints, double *lastSimplex)
{
    double verticesA[24]; //< Not fool proof
    double verticesB[24];
    int vertexCountA, vertexCountB;

    getBodyVertices(a, verticesA, &vertexCountA);
    getBodyVertices(b, verticesB, &vertexCountB);

    if (isConvexBodiesIntersect(
        verticesA,
        vertexCountA,
        verticesB,
        vertexCountB,
        nearestPoints,
        lastSimplex)
    )
    {
        a->colliding = 1;
        b->colliding = 1;
        if (nearestPoints)
        {
            memcpy(
                COL_latestNearestPoints,
                nearestPoints,
                sizeof(COL_latestNearestPoints)
            );
        }
        return 1;
    }
    if (nearestPoints)
    {
        memcpy(
            COL_latestNearestPoints,
            nearestPoints,
            sizeof(COL_latestNearestPoints)
        );
    }
    return 0;
}

