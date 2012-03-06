#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
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
 * Updates the rotation matrix for a body.
 *
 * @param [in,out] body The body to update the rotation of.
 */
void updateRotation(DYN_Body *body)
{
    double angularSpeed = ALG_getVectorLength(body->angularVelocity);
    double axis[3];

    memcpy(axis, body->angularVelocity, sizeof(axis));
    ALG_normalizeVector(axis);
    ALG_createRotationMatrix(body->rotation, axis, angularSpeed);
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
        double bodyAxis[3]; // Axis is in the object space.
        double momentOfInertia;
        double *deltaOmega;

        // Calculate the moment of intertia for the given axis.
        assert(ALG_solveSystem3(bodyAxis, body->orientation, angularImpulse));
        ALG_normalizeVector(bodyAxis);
        // Calculate It * t
        ALG_transform(tmp, bodyAxis, body->staticAttributes->intertiaTensor);
        momentOfInertia = ALG_dotProduct(tmp, bodyAxis);
        assert(fabs(momentOfInertia) > 1e-9);
        // Calculate the change of the angular momentum.
        deltaOmega = angularImpulse;
        ALG_scale(deltaOmega, context->timeStep / momentOfInertia);
        ALG_translate(body->angularVelocity, deltaOmega);
        // Update rotation properties
        updateRotation(body);
    }

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

void DYN_initialize(DYN_Context *context)
{
    context->bodies = 0;
    context->bodiesAllocated = 0;
}

void DYN_deinitialize(DYN_Context *context)
{
    free(context->bodies);
    free(context->staticAttributes);
}

void DYN_stepWorld(DYN_Context *context)
{
    int i;
    // STEP 1: Moving bodies
    for (i = 0; i < context->bodyCount; i++)
    {
        moveBody(context, &context->bodies[i]);
    }
    // STEP 2: Collision detection
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
    }
    context->bodies[context->bodyCount] = *body;
    context->staticAttributes[context->bodyCount] = *attribs;
    context->bodyCount++;
}












