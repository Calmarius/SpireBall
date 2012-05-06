/*!

 @mainpage

 Things to do:

 - Create the EPL engine
    - Create a 3D phisics library.
        - Vector and matrix algebra - DONE!
        - Body movement - DONE!
        - Body dynamics
            - Impulses - DONE!
        - Dynamics part - DONE!
            - Code the calculations when impulse acts on the body. - DONE!
        - Collision detection part.
            - Do convex body collision. - DONE!
            - When does two trimeshes collide?
            - Resolve stucking simulation
                - Use mark stucked pairs so they won't collide. - DONE!
                - Implement EPA to seperate these interpenetrating bodies.
                    - Test the EPA algorithm.
                        - Make EPA capable to run, when starting simplex cannot be given.
        - Optimize calculations
            - How to handle many objects.
                - Broad phase collision detection.
                - Narrow phase collision detection.
                - Prefer continuous collision detection.
    - Write network code.
 - Visualization tool.
 - Make a test program for the library.
 - Create the game UI
 - Artwork
    - Get sounds
    - Get 3D models
        - For spaceships
        - For laser bolts
        - For the balls
        - For the texture on the wall.
 - Create engine test environment.

 */
