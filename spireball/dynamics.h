#ifndef DYNAMICS_H
#define DYNAMICS_H

/**
 * An enum of the possible shapes.
 */
typedef enum DYN_Shape
{
    DYN_BS_CUBOID ///< A cuboid
} DYN_Shape;
/**
 * Represents the static attributes of a body.
 */
typedef struct DYN_BodyStaticAttributes
{
    double mass; ///< Mass of the body.
    double intertiaTensor[9]; ///< Intertia tensor matrix.
    DYN_Shape shape;
    union
    {
        struct
        {
            double width;  ///< Dimension in the X direction
            double height;  ///< Dimension in the Y direction
            double depth;  ///< Dimension in the Z direction
        } cuboidAttributes; ///< Attributes of a cuboid.
    };

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
    double velocity[3]; ///< Velocity of the object (units/timestep)
    /**
     * Angular velocity of the object. The direction of the vector is the rotation axis
     * Its length is the speed of the rotation. (radians/timestep)
     */
    double angularVelocity[3];
    /**
     * Cached rotation matrix. The orientation is multiplied with it.
     */
    double rotation[9];
    /**
     * Pointer to the static attributes of the body.
     */
    DYN_BodyStaticAttributes *staticAttributes;
    /**
     * Nonzero when the body is colliding with something.
     */
    char colliding;
} DYN_Body;

/**
 * Represents a dynaics context. This struct is the `world`.
 */
typedef struct DYN_Context
{
    double timeStep; ///< Fixed timestep value. All objects in the world are moving with this time step.
    DYN_Body *bodies; ///< Array that stores the hot attributes of the bodies.
    int bodyCount; ///< Count of bodies.
    int bodiesAllocated; ///< Allocated space for the bodies.
    DYN_BodyStaticAttributes *staticAttributes; ///< Array that stores the the static attributes of the body.
} DYN_Context;

/**
 * Initializes a dynamics context.
 *
 * @param [in,out] context The context struct.
 * @param [in] timeStep The time step of one step.
 */
void DYN_initialize(DYN_Context *context, double timeStep);
/**
 * Deinitializes the dynamics context. Releases the associated resources.
 *
 * @param [in,out] context
 */
void DYN_deinitialize(DYN_Context *context);
/**
 * Steps the objects in the world. This is the hottest function in this module.
 *
 * @param [in,out] The world.
 */
void DYN_stepWorld(DYN_Context *context);
/**
 * Adds a body to the world.
 *
 * @param [in,out] context The dynamics context.
 * @param [in] body A filled body structure.
 * @param [in] attribs A filled structure of the static attributes of the body.
 */
void DYN_addBody(
    DYN_Context *context,
    const DYN_Body *body,
    const DYN_BodyStaticAttributes *attribs
);

/**
 * Calculates the mass and the inertia tensor of the body.
 *
 * @param [in,out] attributes The attributes of the body, the shape must be set.
 * @param [in] density The density of the
 */
void DYN_calculateMass(DYN_BodyStaticAttributes *attributes, double density);


#endif // DYNAMICS_H
