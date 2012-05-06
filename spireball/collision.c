#include <string.h>
#include <stdlib.h>
#include <assert.h>

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

static int polytopeSideComparer(void *a, void *b)
{
    PolytopeSide *aside = (PolytopeSide*)a;
    PolytopeSide *bside = (PolytopeSide*)b;
    if (aside->squaredDistance < bside->squaredDistance) return -1;
    if (aside->squaredDistance > bside->squaredDistance) return 1;
    return 0;
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

    assert(isPointInTetraHedron(nullVector, startingSimplex));
    AVL_initialize(&treeIndex, polytopeSideComparer);
    DYNA_initialize(&dynArray, sizeof(PolytopeSide));
    // Add the initial triangles
    {
        int i,j;
        // 4 triangles
        for (i = 0; i < 4; i++)
        {
            int k = 0;
            PolytopeSide tmp;
            // Get a triange by taking the vertices of the tetraheadron, except the i-th.
            for (j = 0; j < 4; j++)
            {
                if (i == j) continue;
                memcpy(
                    &tmp.vertices[k * 3],
                    &startingSimplex[j * 3],
                    3 * sizeof(double)
                );
                k++;
            }
            if (getSupportVectorOfTriangle(tmp.vertices, tmp.supportVector, 0, 0))
            {
                void *added;

                tmp.squaredDistance = ALG_dotProduct(
                    tmp.supportVector,
                    tmp.supportVector
                );
                added = DYNA_add(&dynArray, &tmp);
                AVL_add(&treeIndex, added, 0);
            }
        }
    }
    // Do EPA here.
    for(;;)
    {
        // Get the face which is nearest to the origin.
        AVL_Node *least = AVL_getLeast(&treeIndex);
        PolytopeSide *pside = AVL_getKey(least);
        double newVertex[3];
        PolytopeSide newSide;
        int i, j;

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
        // Add new triangles to the index.
        // 3 new triangles.
        for (i = 0; i < 3; i++)
        {
            int k = 0;
            // 2 vertex from the old triangle
            for (j = 0; j < 3; j++)
            {
                if (i == j) continue;
                memcpy(&newSide.vertices[3 * k], &pside->vertices[3 * j], 3 * sizeof(double));
                k++;
            }
            // plus the new vertex
            memcpy(&newSide.vertices[6], newVertex, 3 * sizeof(double));
            // Calculate support vector on these triangles
            if (getSupportVectorOfTriangle(newSide.vertices, newSide.supportVector, 0, 0))
            {
                void *added;
                newSide.squaredDistance = ALG_dotProduct(
                    newSide.supportVector,
                    newSide.supportVector
                );
                added = DYNA_add(&dynArray, &newSide);
                AVL_add(&treeIndex, added, 0);
            }
        }
        // Remove the checked triangle
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

