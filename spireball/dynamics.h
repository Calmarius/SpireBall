#ifndef DYNAMICS_H
#define DYNAMICS_H

/**
 * Represents the static attributes of a body.
 */
typedef struct DYN_BodyStaticAttributes
{
    double mass; ///< Mass of the body.
    double intertiaTensor[9]; ///< Intertia tensor matrix.
} DYN_BodyStaticAttributes;

/**
 * Represents a body.
 */
typedef struct DYN_Body
{
    /**
     * A matrix in row major order that stores the X, Y and Z directions of the object.
     */
    double orientation[9];
    double position[3]; ///< Position of the object
    double velocity[3]; ///< Velocity of the object
    /**
     * Angular velocity of the object. The direction of the vector is the rotation axis
     * Its length is the speed of the rotation.
     */
    double angularVelocity[3];
    /**
     * Pointer to the static attributes of the body.
     */
    DYN_BodyStaticAttributes *staticAttributes;
} DYN_Body;


#endif // DYNAMICS_H
