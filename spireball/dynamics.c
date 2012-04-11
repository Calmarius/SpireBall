#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "dynamics.h"
#include "algebra.h"
#include "collision.h"

const double DYN_IDENTITY[9] =
{
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};

FILE *DYN_log = 0;


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
 * Returns the rotation matrix, that rotates body in the given fraction of the timestep.
 *
 * @param [in] body The body to query.
 * @param [in,out] rotation The resulting rotation matrix
 * @param [in] timeStepFactor The factor of one time step. 0 means no rotation.
 *      1 means full timestep, between 0 and 1 means fraction of time step.
 */
void getRotation(const DYN_Body *body, double *rotation, double timeStepFactor)
{
    double angularSpeed = ALG_getVectorLength(body->angularVelocity) * timeStepFactor;
    double axis[3];

    memcpy(axis, body->angularVelocity, sizeof(axis));
    if (ALG_isNullVector(axis))
    {
        memcpy(rotation, DYN_IDENTITY, sizeof(DYN_IDENTITY));
    }
    else
    {
        ALG_normalizeVector(axis);
        ALG_createRotationMatrix(rotation, axis, angularSpeed);
    }
}

/**
 * Updates the rotation matrix for a body.
 *
 * @param [in,out] body The body to update the rotation of.
 */
void updateRotation(DYN_Body *body)
{
    getRotation(body, body->rotation, 1);
}

/**
 * Calculates the moment of inertia for the current rotation axis of the body.
 *
 * @param [in,out] body The body to update.
 */
void updateMomentOfInertia(DYN_Body *body)
{
    double tmp[3];
    double rotationAxis[3]; // rotation axis in body space.

    ALG_solveSystem3(rotationAxis, body->orientation, body->angularVelocity);
    ALG_normalizeVector(rotationAxis);
    ALG_transform(tmp, rotationAxis, body->staticAttributes->intertiaTensor);
    body->momentOfInertia = ALG_dotProduct(rotationAxis, tmp);
}

/**
 * Updates the body's angular velocity and rotation from the given angular momentum vector.
 *
 * @param [in,out] body The body to update.
 * @param [in] angularMomentum angular momentum vector.
 */
void updateAngularVelocity(DYN_Body *body, const double *angularMomentum)
{
    double tmp[3];
    double momentOfInertia;
    double rotationAxis[3];

    memcpy(tmp, angularMomentum, sizeof(tmp));
    ALG_solveSystem3(rotationAxis, body->orientation, angularMomentum);
    ALG_normalizeVector(rotationAxis);
    ALG_transform(tmp, rotationAxis, body->staticAttributes->intertiaTensor);
    momentOfInertia = ALG_dotProduct(rotationAxis, tmp);
    assert(fabs(momentOfInertia) > 1e-9);
    memcpy(body->angularVelocity, angularMomentum, sizeof(*angularMomentum) * 3);
    ALG_scale(body->angularVelocity, 1.0/momentOfInertia);
    body->momentOfInertia = momentOfInertia;

    updateRotation(body);
}


/**
 * Applies impulse to a body.
 *
 * @param [in] context The context the body is in.
 * @param [in,out] body The body to apply the impulse to.
 * @param [in] pointOfForce The point where the force applied.
 * @param [in] impulseVector The direction and strength of the impulse (in N * time_step)
 *
 */
void applyImpulse(
    DYN_Context *context,
    DYN_Body *body,
    const double *pointOfForce,
    const double *impulseVector
)
{
    double radius[3]; // vector from the mass center to the point where the force applies.
    double angularImpulse[3]; // The angular impulse applied to the body.
    double tmp[3];

    ALG_getPointToPointVector(radius, body->position, pointOfForce);
    ALG_crossProduct(angularImpulse, radius, impulseVector);
    // Apply the change to the linear momentum
    memcpy(tmp, impulseVector, sizeof(tmp));
    ALG_scale(tmp, 1 / body->staticAttributes->mass);
    ALG_translate(body->velocity, tmp);
    if (!ALG_isNullVector(angularImpulse))
    {
        double bodyRotationAxis[3]; // Axis of the angular impulse is in the object space.
        double momentOfInertiaCurrentAxis;
        double angularMomentum[3];

        // Calculate the moment of inertia for the current rotational axis
        assert(ALG_solveSystem3(bodyRotationAxis, body->orientation, body->angularVelocity));
        ALG_normalizeVector(bodyRotationAxis);
        // Calculate Io * o
        ALG_transform(tmp, bodyRotationAxis, body->staticAttributes->intertiaTensor);
        momentOfInertiaCurrentAxis = ALG_dotProduct(tmp, bodyRotationAxis);
        assert(momentOfInertiaCurrentAxis >= 0);
        // Calculate the current angular momentum of the body
        memcpy(angularMomentum, body->angularVelocity, sizeof(tmp));
        ALG_scale(angularMomentum, momentOfInertiaCurrentAxis); // current angular momentum
        ALG_translate(angularMomentum, angularImpulse); // new angular momentum.
        // Update angular velocity and rotation.
        updateAngularVelocity(body, angularMomentum);
    }
    //fprintf(f, "\n");

}

/**
 * Reverts the previous movement of the body.
 *
 * @param [in] body The body to revert.
 */
void revertMovement(DYN_Body *body)
{
    memcpy(body->position, body->prevPosition, sizeof(body->position));
    memcpy(body->orientation, body->prevOrientation, sizeof(body->orientation));
}

/**
 * Moves the body using its attributes.
 *
 * @param [in,out] body The body to move.
 * @param [in] subStepFactor It's possible move the body in a smaller step than the time step
 *      set in the context, this is slower. 0 means no movement, return immedately, 1 means
 *      full step, which is fast, because the rotation is cached. Otherwise the calculation is slower.
 */
void moveBody(DYN_Body *body, double subStepFactor)
{
    double newOrientation[9];

    if (subStepFactor == 0) return;
    if (subStepFactor == 1)
    {
        // Translate the body.
        memcpy(body->prevPosition, body->position, sizeof(body->position));
        ALG_translate(body->position, body->velocity);
        // Rotate the body
        ALG_multiplyMatrix(newOrientation, body->rotation, body->orientation);
        memcpy(body->prevOrientation, body->orientation, sizeof(body->orientation));
        memcpy(body->orientation, newOrientation, sizeof(newOrientation));
    }
    else
    {
        double stepVelocity[3];
        double stepRotation[9];

        // Translate the body.
        memcpy(body->prevPosition, body->position, sizeof(body->position));
        memcpy(stepVelocity, body->velocity, sizeof(stepVelocity));
        ALG_scale(stepVelocity, subStepFactor);
        ALG_translate(body->position, stepVelocity);
        // Rotate the body
        getRotation(body, stepRotation, subStepFactor);
        ALG_multiplyMatrix(newOrientation, stepRotation, body->orientation);
        memcpy(body->prevOrientation, body->orientation, sizeof(body->orientation));
        memcpy(body->orientation, newOrientation, sizeof(newOrientation));
    }
}

void DYN_initialize(DYN_Context *context, double timeStep)
{
    context->bodies = 0;
    context->bodiesAllocated = 0;
    context->timeStep = timeStep;
    context->collidingPairsAllocated = 0;
    context->collidingPairCount = 0;
    context->elapsedTime = 0;
    if (!DYN_log)
    {
        DYN_log = fopen("log.txt", "wt");
    }
}

void DYN_deinitialize(DYN_Context *context)
{
    free(context->bodies);
    free(context->staticAttributes);
}

void addCollidingPair(DYN_Context *context, int a, int b)
{
    if (context->collidingPairCount == context->collidingPairsAllocated)
    {
        if (!context->collidingPairsAllocated)
        {
            context->collidingPairsAllocated = 100;
        }
        else
        {
            context->collidingPairsAllocated <<= 1;
        }
        context->collidingBodyPairs = realloc(
            context->collidingBodyPairs,
            context->collidingPairsAllocated * sizeof(*context->collidingBodyPairs)
        );
    }
    context->collidingBodyPairs[context->collidingPairCount][0] = a;
    context->collidingBodyPairs[context->collidingPairCount][1] = b;
    context->collidingPairCount++;
}

void clearCollidingPairs(DYN_Context *context)
{
    context->collidingPairCount = 0;
}

typedef enum
{
    DYN_COLLIDED,
    DYN_SEPARATING,
    DYN_STICKTOGETHER
} CollisionResult;

CollisionResult resolveCollision
(
    DYN_Context *context,
    DYN_Body *a,
    DYN_Body *b,
    double *collisionPoint,
    double *impulseDirection
)
{
    double radiusA[3];
    double radiusB[3];
    double collisionPointVelocityA[3];
    double collisionPointVelocityB[3];
    double normal[3]; // The impulse's direction A gives B.
    double tmp[3];
    double relativeCollisionPointVelocity[3];
    double impulseStrength;
    double elasticity = 0.1;
    double normalComponent;
    double tangentialComponent;
    const double CRITICAL_SLOW_SPEED = 0.001;
    char isSeparatingSlowly;


    // Get a normalized unit vector
    memcpy(normal, impulseDirection, sizeof(normal));
    ALG_normalizeVector(normal);
    // Get vectors drawn from mass center to the collision point.
    ALG_getPointToPointVector(radiusA, a->position, collisionPoint);
    ALG_getPointToPointVector(radiusB, b->position, collisionPoint);
    // Calculate the speed of the collision points on the bodies.
    memcpy(collisionPointVelocityA, a->velocity, sizeof(collisionPointVelocityA));
    ALG_crossProduct(tmp, a->angularVelocity, radiusA);
    ALG_translate(collisionPointVelocityA, tmp);
    memcpy(collisionPointVelocityB, b->velocity, sizeof(collisionPointVelocityB));
    ALG_crossProduct(tmp, b->angularVelocity, radiusB);
    ALG_translate(collisionPointVelocityB, tmp);
    // Get relative collision point velocity
    ALG_getPointToPointVector(
        relativeCollisionPointVelocity,
        collisionPointVelocityA,
        collisionPointVelocityB
    );
    normalComponent = ALG_dotProduct(normal, relativeCollisionPointVelocity);
    tangentialComponent = sqrt(
        ALG_getVectorLength(relativeCollisionPointVelocity) -
        normalComponent*normalComponent
    );
    isSeparatingSlowly = (normalComponent * -elasticity) < CRITICAL_SLOW_SPEED;
    if (isSeparatingSlowly)
    {
        fprintf(DYN_log, "The two bodies are separating slowly!\n");
    }
    fprintf(DYN_log, "Normal component of relative speed: %g\n", normalComponent);
    fprintf(
        DYN_log,
        "Tangential component of relative speed: %g\n",
        tangentialComponent
    );
/*    if (tangentialComponent != 0)
    {
        if (fabs(normalComponent) < CRITICAL_SLOW_SPEED)
        {
            // Too slow relative normal component may cause the simulation get stuck.
            // In this case, cheat a bit: apply bigger impulse than needed.
            elasticity = 10 * CRITICAL_SLOW_SPEED / fabs(normalComponent);
            // With this elastivity the ratio of the normal and tangential will be at least the
            // arbitrarily chosen critical ratio.
            fprintf(DYN_log, "Elasticity risen to: %g due to small normal component\n", elasticity);
        }
    }*/
    if (normalComponent > 0)
    {
        // In this case, the two bodies separating, they cannot collide.
        return DYN_SEPARATING;
    }
    // Calculate the strength of the impulse
    {
        double tmp[3];
        double tmp2[3];
        double tmpVectorA[3];
        double tmpVectorB[3];

        ALG_crossProduct(tmp, radiusA, normal);
        ALG_transform(tmp2, tmp, a->staticAttributes->inverseInertiaTensor);
        ALG_crossProduct(tmpVectorA, tmp2, radiusA);

        ALG_crossProduct(tmp, radiusB, normal);
        ALG_transform(tmp2, tmp, b->staticAttributes->inverseInertiaTensor);
        ALG_crossProduct(tmpVectorB, tmp2, radiusB);

        /*double radiusAMatrix[9];
        double radiusBMatrix[9];
        double tmpMatrixA[9];
        double tmpMatrixB[9];
        double tmp[9];
        double tmpVectorA[3];
        double tmpVectorB[3];
        double termA, termB;

        ALG_createCrossProductMatrix(radiusAMatrix, radiusA);
        ALG_createCrossProductMatrix(radiusBMatrix, radiusB);

        ALG_multiplyMatrix(tmp, radiusAMatrix, a->staticAttributes->inverseInertiaTensor);
        ALG_multiplyMatrix(tmpMatrixA, tmp, radiusAMatrix);

        ALG_multiplyMatrix(tmp, radiusBMatrix, b->staticAttributes->inverseInertiaTensor);
        ALG_multiplyMatrix(tmpMatrixB, tmp, radiusBMatrix);

        ALG_transform(tmpVectorA, normal, tmpMatrixA);
        ALG_transform(tmpVectorB, normal, tmpMatrixB);

        termA = ALG_dotProduct(tmpVectorA, normal);
        termB = ALG_dotProduct(tmpVectorB, normal);*/

        impulseStrength =
            (-(1 + elasticity)*ALG_dotProduct(relativeCollisionPointVelocity, normal) )/
            (1/a->staticAttributes->mass + 1/b->staticAttributes->mass +
                ALG_dotProduct(tmpVectorA, normal) +
                ALG_dotProduct(tmpVectorB, normal)
            )
            ;

    }
    ALG_scale(normal, impulseStrength);
    applyImpulse(context, b, collisionPoint, normal);
    ALG_scale(normal, -1);
    applyImpulse(context, a, collisionPoint, normal);
    if (isSeparatingSlowly)
    {
        return DYN_STICKTOGETHER;
    }
    else
    {
        return DYN_COLLIDED;
    }
}

char simStopped = 0;

double DYN_lastCollisionPoint[3];
double DYN_lastImpulse[3];

double *DYN_getLastImpulse() {return DYN_lastImpulse;}
double *DYN_getLastCollisionPoint() {return DYN_lastCollisionPoint;}

void DYN_stepWorld(DYN_Context *context)
{
    int i, j;
    double remainingTime = 1;
    double nearestPoints[6];
    int stuckCtr = 0;

    if (simStopped) return;

    while (remainingTime > 0)
    {
        char wasCollision = 0;
        // STEP 1: Moving bodies
        for (i = 0; i < context->bodyCount; i++)
        {
            moveBody(&context->bodies[i], remainingTime);
        }
        // STEP 2: Collision detection
        for (i = 0; i < context->bodyCount; i++)
        {
            context->bodies[i].colliding = 0;
        }
        for (i = 0; i < context->bodyCount; i++)
        {
            for (j = i + 1; j < context->bodyCount; j++)
            {
                double nearestPoints[6];
                if (COL_collide(&context->bodies[i], &context->bodies[j], nearestPoints))
                {
                    addCollidingPair(context, i, j);
                    wasCollision = 1;
                }
            }
        }
        if (wasCollision)
        {
            double currentImpulseVector[3];
            double currentCollisionPoint[3];
            double minTime = 1;
            int earlyAIndex = -1, earlyBIndex = -1;
            // There was collision
            // STEP 3: Find earliest collision
            int i;
            // Find the earliest collision.
            assert(context->collidingPairCount);
            for (i = 0; i < context->collidingPairCount; i++)
            {
                DYN_Body *a = &context->bodies[context->collidingBodyPairs[i][0]];
                DYN_Body *b = &context->bodies[context->collidingBodyPairs[i][1]];
                double lower = 0;
                double upper = remainingTime;
                int stuckCtr2 = 0;
                char nearestSet = 0;

                for(;;stuckCtr2++)
                {
                    double middle = (lower + upper) * 0.5;

                    revertMovement(a);
                    revertMovement(b);
                    moveBody(a, middle);
                    moveBody(b, middle);
                    if (COL_collide(a, b, nearestPoints))
                    {
                        upper = middle;
                    }
                    else
                    {
                        nearestSet = 1;
                        lower = middle;
                    }
                    assert(lower != upper);
                    if (nearestSet)
                    {
                        double tmp[3];
                        ALG_getPointToPointVector(tmp, nearestPoints, &nearestPoints[3]);
                        if ((ALG_dotProduct(tmp, tmp) < 1e-6) || (fabs(upper-lower) < 1e-6))
                        {
                            // The two bodies are near each other.
                            if (middle <= minTime)
                            {
                                minTime = middle;
                                earlyAIndex = context->collidingBodyPairs[i][0];
                                earlyBIndex = context->collidingBodyPairs[i][1];
                                memcpy(currentImpulseVector, tmp, sizeof(currentImpulseVector));
                                memcpy(currentCollisionPoint, nearestPoints, sizeof(currentCollisionPoint));
                            }
                            break;
                        }
                    }
                }
            }
            {
                DYN_Body *a = &context->bodies[earlyAIndex];
                DYN_Body *b = &context->bodies[earlyBIndex];
                fprintf(DYN_log, "Collision!\n");
                fprintf(DYN_log, "Body index A: %d\n", earlyAIndex);
                fprintf(DYN_log, "Body index B: %d\n", earlyBIndex);
                fprintf(DYN_log, "Velocity A [%g, %g, %g] %g\n", a->velocity[0], a->velocity[1], a->velocity[2], ALG_getVectorLength(a->velocity));
                fprintf(DYN_log, "Velocity B [%g, %g, %g] %g\n", b->velocity[0], b->velocity[1], b->velocity[2], ALG_getVectorLength(b->velocity));
                fprintf(DYN_log, "Collision point [%g, %g, %g]\n", currentCollisionPoint[0], currentCollisionPoint[1], currentCollisionPoint[2]);
                fprintf(
                    DYN_log,
                    "Impulse vector [%g, %g, %g] %g\n",
                    currentImpulseVector[0],
                    currentImpulseVector[1],
                    currentImpulseVector[2],
                    ALG_getVectorLength(currentImpulseVector)
                );
                fprintf(DYN_log, "Elapsed time: %g\n", context->elapsedTime + context->timeStep * (1 - remainingTime));
            }
            // Revert and move all bodies to the collision moment.
            for (i = 0; i < context->bodyCount; i++)
            {
                DYN_Body *current = &context->bodies[i];
                revertMovement(current);
                moveBody(current, minTime);
            }
            assert(earlyAIndex >= 0);
            assert(earlyBIndex >= 0);
            // Resolve the collision
             resolveCollision(
                context,
                &context->bodies[earlyAIndex],
                &context->bodies[earlyBIndex],
                currentCollisionPoint,
                currentImpulseVector
            );
            memcpy(DYN_lastCollisionPoint, currentCollisionPoint, sizeof(DYN_lastCollisionPoint));
            memcpy(DYN_lastImpulse, currentImpulseVector, sizeof(DYN_lastImpulse));
            fprintf(DYN_log, "==============================\n");
            //
            clearCollidingPairs(context);
            // Continue the simulation
            remainingTime -= minTime;
        }
        else
        {
            // No collision, step finished
            break;
        }
        stuckCtr++;
        if (stuckCtr >= 100)
        {
            simStopped = 1;
            return;
        }
    }
    context->elapsedTime += context->timeStep;
}

void DYN_addBody(
    DYN_Context *context,
    const DYN_Body *body,
    const DYN_BodyStaticAttributes *attribs
)
{
    if (context->bodyCount == context->bodiesAllocated)
    {
        if (!context->bodiesAllocated)
        {
            context->bodiesAllocated = 20;
        }
        else
        {
            context->bodiesAllocated <<= 1;
        }
        context->bodies = realloc(
            context->bodies,
            sizeof(DYN_Body) * context->bodiesAllocated
        );
        context->staticAttributes = realloc(
            context->staticAttributes,
            sizeof(DYN_BodyStaticAttributes) * context->bodiesAllocated
        );
        // Update static attribute pointers
        {
            int i;
            for (i = 0; i < context->bodyCount; i++)
            {
                context->bodies[i].staticAttributes = &context->staticAttributes[i];
            }
        }
    }
    context->bodies[context->bodyCount] = *body;
    context->staticAttributes[context->bodyCount] = *attribs;
    {
        DYN_Body *addedbody = &context->bodies[context->bodyCount];
        addedbody->staticAttributes = &context->staticAttributes[context->bodyCount];
        addedbody->colliding = 0;
        updateMomentOfInertia(addedbody);
        updateRotation(addedbody);
    }
    context->bodyCount++;
}

/**
 * Calculates the mass and inertia tensor of a cuboid.
 *
 * @param [in,out] attributes The preset attributes with the shape of the cuboid.
 * @param [in] denstiry The density of the cuboid
 */
void calculateCuboidMass(DYN_BodyStaticAttributes *attributes, double density)
{
    assert(attributes->shape == DYN_BS_CUBOID);
    double h = attributes->cuboidAttributes.height;
    double w = attributes->cuboidAttributes.width;
    double d = attributes->cuboidAttributes.depth;
    double volume = h * w * d;
    attributes->mass = volume * density;
    memset(attributes->intertiaTensor, 0, sizeof(*attributes->intertiaTensor) * 9);
    attributes->intertiaTensor[0] = 1.0/12.0 * attributes->mass * (h*h + d*d);
    attributes->intertiaTensor[4] = 1.0/12.0 * attributes->mass * (w*w + d*d);
    attributes->intertiaTensor[8] = 1.0/12.0 * attributes->mass * (h*h + w*w);
}

void DYN_calculateMass(DYN_BodyStaticAttributes *attributes, double density)
{
    switch (attributes->shape)
    {
        case DYN_BS_CUBOID:
            calculateCuboidMass(attributes, density);
        break;
        default:
            assert(0); //< Unsupported shape
    }
    ALG_invertMatrix(attributes->inverseInertiaTensor, attributes->intertiaTensor);
}


int castRayOnCuboid(
    const DYN_Body *body,
    const double *point,
    const double *direction,
    double *length
)
{
    /*
     * The hit point of the ray can be described with the following equation:
     *
     * M + ku + v + lw = P + mr
     *
     * Where:
     *
     * M: center of mass
     * u, v, w: 3 prependicular vectors from center to teh center of 3 faces.
     * P: point where the ray is shot from
     * r: the direction of the ray.
     * k, l, m: unknowns.
     *
     * Solve the equation for k, l, m. Intersection with the face is valid if
     *
     * |k|,|l| <= 1 and m >=0
     *
     * After reordering the equation we get:
     *
     * ku + lw - mr = P - M - v
     *
     * This is the equation for a single side.
     *
     * We also need to solve the equations for all faces:
     *
     * ku + lw - mr = P - M - v
     * ku + lw - mr = P - M + v
     * ku + lv - mr = P - M - w
     * ku + lv - mr = P - M + w
     * kv + lw - mr = P - M - u
     * kv + lw - mr = P - M + u
     *
     */

/*    double point[3] = {0, 10, 0};
    double direction[3] = {0, -1, 0};*/

    double orientationVectors[9];
    DYN_BodyStaticAttributes *staticAttributes = body->staticAttributes;
    double matrix[9]; // matrix of the equation.
    double basicVector[3]; // point - mass center
    double rightSide[3]; // The right side of the equation.
    int i, j, k, l;
    double solution[3]; // coefficients
    double shortest;
    int rayExist = 0;

    assert(staticAttributes->shape == DYN_BS_CUBOID);

    ALG_getPointToPointVector(basicVector, body->position, point); // P - M

    memcpy(orientationVectors, body->orientation, sizeof(orientationVectors)); // u, v, w
    // Make row vectors from it.
    ALG_transposeMatrix(orientationVectors);
    // Scale the vectors to the appropriate length.
    ALG_scale(&orientationVectors[0], staticAttributes->cuboidAttributes.width * 0.5);
    ALG_scale(&orientationVectors[3], staticAttributes->cuboidAttributes.height * 0.5);
    ALG_scale(&orientationVectors[6], staticAttributes->cuboidAttributes.depth * 0.5);

    matrix[2] = -direction[0]; // -r
    matrix[5] = -direction[1];
    matrix[8] = -direction[2];

    // Finding intersection points with the side planes of the cuboid.
    for (i = 0; i < 3; i++)
    {
        for (k = -1; k <= 1; k += 2)
        {
            l = 0;
            for (j = 0; j < 3; j++)
            {
                if (i == j)
                {
                    memcpy(rightSide, &orientationVectors[3*i], sizeof(rightSide)); // The third term of the right side.
                    ALG_scale(rightSide, k); // negate it when needed.
                    ALG_translate(rightSide, basicVector); // P - M + 3rd_term
                }
                else
                {
                    // Set the other two vectors in the matrix.
                    matrix[l] = orientationVectors[3*j];
                    matrix[3 + l] = orientationVectors[3*j + 1];
                    matrix[6 + l] = orientationVectors[3*j + 2];
                    l++;
                }
            }
            // Solve the current equation
            if (!ALG_solveSystem3(solution, matrix, rightSide))
            {
                continue;
            }
            if (
                (solution[0] >= -1) && ( solution[0] <= 1) &&
                (solution[1] >= -1) && ( solution[1] <= 1) &&
                (solution[2] >= 0)
            )
            {
                // This is a hit on the current side.
                if (rayExist)
                {
                    if (shortest < solution[2])
                    {
                        shortest = solution[2];
                    }
                }
                else
                {
                    rayExist = 1;
                    shortest = solution[2];
                }
            }
        }
    }
    if (rayExist)
    {
        *length = solution[2];
    }
    return rayExist;
}

int DYN_castRay(
    const DYN_Body *body,
    const double *point,
    const double *direction,
    double *length
)
{
    switch (body->staticAttributes->shape)
    {
        case DYN_BS_CUBOID:
            return castRayOnCuboid(body, point, direction, length);
        break;
        default:
            assert(0);
        break;
    }
}










