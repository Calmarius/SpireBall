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
 * Updates the rotation matrix for a body.
 *
 * @param [in,out] body The body to update the rotation of.
 */
void updateRotation(DYN_Body *body)
{
    double angularSpeed = ALG_getVectorLength(body->angularVelocity);
    double axis[3];

    memcpy(axis, body->angularVelocity, sizeof(axis));
    if (ALG_isNullVector(axis))
    {
        memcpy(body->rotation, DYN_IDENTITY, sizeof(DYN_IDENTITY));
    }
    else
    {
        ALG_normalizeVector(axis);
        ALG_createRotationMatrix(body->rotation, axis, angularSpeed);
    }
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
 * @param [in] impulseVector The direction and strength of the impulse (in Ns)
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
    ALG_scale(tmp, context->timeStep / body->staticAttributes->mass);
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
        // Calculate the current angular impulse of the body
        memcpy(angularMomentum, body->angularVelocity, sizeof(tmp));
        ALG_scale(angularMomentum, momentOfInertiaCurrentAxis); // current angular momentum
        ALG_scale(angularImpulse, context->timeStep);
        ALG_translate(angularMomentum, angularImpulse); // new angular momentum.
        // Update angular velocity and momentum and rotation.
        updateAngularVelocity(body, angularMomentum);
    }
    //fprintf(f, "\n");

}

/**
 * Moves the body using its attributes.
 *
 * @param [in,out] body The body to move.
 * @param [in] timeStep The time interval.
 */
void moveBody(DYN_Context *context, DYN_Body *body)
{
    double newOrientation[9];
    // Translate the body.
    ALG_translate(body->position, body->velocity);
    // Rotate the body
    ALG_multiplyMatrix(newOrientation, body->rotation, body->orientation);
    memcpy(body->orientation, newOrientation, sizeof(newOrientation));
}

void DYN_initialize(DYN_Context *context, double timeStep)
{
    context->bodies = 0;
    context->bodiesAllocated = 0;
    context->timeStep = timeStep;
}

void DYN_deinitialize(DYN_Context *context)
{
    free(context->bodies);
    free(context->staticAttributes);
}

void DYN_stepWorld(DYN_Context *context)
{
    int i, j;
    // STEP 1: Moving bodies
    for (i = 0; i < context->bodyCount; i++)
    {
        moveBody(context, &context->bodies[i]);
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
            COL_collide(&context->bodies[i], &context->bodies[j]);
        }
    }
    // STEP 3: Collsion resolution
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










