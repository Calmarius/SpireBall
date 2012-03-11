#ifdef __APPLE__
#include <SDL/SDL.h>
#else
#include <SDL.h>
#endif
#include <unistd.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "../spireball/algebra.h"
#include "../spireball/dynamics.h"

double camPosition[3] = {0, 0, 0};
double camOrientation[9] =
{
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
};
struct
{
    char moveForward;
    char moveBackward;
    char moveLeft;
    char moveRight;
    char moveUp;
    char moveDown;

    char turnLeft;
    char turnRight;
    char pitchUp;
    char pitchDown;
    char bankLeft;
    char bankRight;
} movement = {0};
char mouseGrabbed = 0;
int mouseXRef, mouseYRef; // Reference mouse pos.
int mouseXCurrent, mouseYCurrent;
DYN_Context world;

GLuint cubeModel;

void convertDynMatrixToGLMatrix(const double *dynMatrix, double *oglMatrix)
{
    memset(oglMatrix, 0, sizeof(*oglMatrix) * 16);
    oglMatrix[0] = dynMatrix[0];
    oglMatrix[4] = dynMatrix[1];
    oglMatrix[8] = dynMatrix[2];
    oglMatrix[1] = dynMatrix[3];
    oglMatrix[5] = dynMatrix[4];
    oglMatrix[9] = dynMatrix[5];
    oglMatrix[2] = dynMatrix[6];
    oglMatrix[6] = dynMatrix[7];
    oglMatrix[10] = dynMatrix[8];
    oglMatrix[15] = 1;
}

void drawCuboid(const DYN_Body *body, const DYN_BodyStaticAttributes *attributes)
{
    double orientationMatrix[16];

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    // Do matrix calculate transformation matrix.
    // Transformations are done in reverse order.
    // tranlate
    glTranslated(body->position[0], body->position[1], body->position[2]);
    // rotate
    convertDynMatrixToGLMatrix(body->orientation, orientationMatrix);
    glMultMatrixd(orientationMatrix);
    // Scale the cube
    glScaled(
        attributes->cuboidAttributes.width * 0.5,
        attributes->cuboidAttributes.height * 0.5,
        attributes->cuboidAttributes.depth * 0.5
    );
    glCallList(cubeModel);
    glPopMatrix();
}

void drawBody(const DYN_Body *body)
{
    switch (body->staticAttributes->shape)
    {
        case DYN_CUBOID:
        {
            glColor3f(1,1,1);
            drawCuboid(body, body->staticAttributes);
        }
        break;
    }
}

void drawBodies()
{
    int i;
    for (i = 0; i < world.bodyCount; i++)
    {
        drawBody(&world.bodies[i]);
    }
}

void drawAxes()
{
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    {
        glColor3f(0, 0, 0);
        glVertex3f(0, 0, 0);
        glColor3f(1, 0, 0);
        glVertex3f(1, 0, 0);

        glColor3f(0, 0, 0);
        glVertex3f(0, 0, 0);
        glColor3f(0, 1, 0);
        glVertex3f(0, 1, 0);

        glColor3f(0, 0, 0);
        glVertex3f(0, 0, 0);
        glColor3f(0, 0, 1);
        glVertex3f(0, 0, 1);
    }
    glEnd();
    glPopAttrib();
}

void drawGrid()
{
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glColor3f(0.0, 0.3, 0.0);
    glLineWidth(1);
    glBegin(GL_LINES);
    {
        int i;
        for (i = -50; i < 50; i++)
        {
            if (i % 5 == 0)
            {
                continue;
            }
            glVertex3f(i, 0, -50);
            glVertex3f(i, 0, 50);
            glVertex3f(-50, 0, i);
            glVertex3f(50, 0, i);
        }
    }
    glEnd();
    glLineWidth(3);
    glBegin(GL_LINES);
    {
        int i;
        for (i = -50; i < 50; i+=5)
        {
            if (!i)
            {
                continue;
            }
            glVertex3f(i, 0, -50);
            glVertex3f(i, 0, 50);
            glVertex3f(-50, 0, i);
            glVertex3f(50, 0, i);
        }
    }
    glEnd();
    glLineWidth(5);
    glBegin(GL_LINES);
    {
        glVertex3f(0, 0, -50);
        glVertex3f(0, 0, 50);
        glVertex3f(-50, 0, 0);
        glVertex3f(50, 0, 0);
    }
    glEnd();
    glPopAttrib();
}

void draw()
{
    GLfloat position[4] = {1, 1, 1, 0};

    // Setting camera

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(
        camPosition[0],
        camPosition[1],
        camPosition[2],
        camPosition[0] - camOrientation[2],
        camPosition[1] - camOrientation[5],
        camPosition[2] - camOrientation[8],
        camOrientation[1],
        camOrientation[4],
        camOrientation[7]
    );

    // Setting light position

    glLightfv(GL_LIGHT0, GL_POSITION, position);

    // Drawing
    glClearColor(0, 0, 0.0f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawAxes();
    drawGrid();
    drawBodies();
}

char handleEvents()
{
    char done = 0;
    SDL_Event event;
    while (SDL_PollEvent(&event))
    {
        // check for messages
        switch (event.type)
        {
            // exit if the window is closed
        case SDL_QUIT:
            done = 1;
        break;

            // check for keypresses
        case SDL_KEYDOWN:
            {
                if (event.key.keysym.sym == SDLK_ESCAPE)
                {
                    done = 1;
                }
                switch (event.key.keysym.sym)
                {
                    case SDLK_a: movement.moveLeft = 1; break;
                    case SDLK_d: movement.moveRight = 1; break;
                    case SDLK_w: movement.moveForward = 1; break;
                    case SDLK_s: movement.moveBackward = 1; break;
                    case SDLK_e: movement.moveUp = 1; break;
                    case SDLK_q: movement.moveDown = 1; break;
                    case SDLK_LALT: movement.bankLeft = 1; break;
                    case SDLK_SPACE: movement.bankRight = 1; break;
                    case SDLK_LEFT: movement.turnLeft = 1; break;
                    case SDLK_RIGHT: movement.turnRight = 1; break;
                    case SDLK_UP: movement.pitchUp = 1; break;
                    case SDLK_DOWN: movement.pitchDown = 1; break;
                    default: break;
                }
            }
        break;
        case SDL_KEYUP:
            {
                switch (event.key.keysym.sym)
                {
                    case SDLK_a: movement.moveLeft = 0; break;
                    case SDLK_d: movement.moveRight = 0; break;
                    case SDLK_w: movement.moveForward = 0; break;
                    case SDLK_s: movement.moveBackward = 0; break;
                    case SDLK_e: movement.moveUp = 0; break;
                    case SDLK_q: movement.moveDown = 0; break;
                    case SDLK_LALT: movement.bankLeft = 0; break;
                    case SDLK_SPACE: movement.bankRight = 0; break;
                    case SDLK_LEFT: movement.turnLeft = 0; break;
                    case SDLK_RIGHT: movement.turnRight = 0; break;
                    case SDLK_UP: movement.pitchUp = 0; break;
                    case SDLK_DOWN: movement.pitchDown = 0; break;
                    default: break;
                }
            }
        break;
        case SDL_MOUSEBUTTONDOWN:
            if (event.button.button == SDL_BUTTON_LEFT)
            {
                mouseGrabbed = 1;
                mouseXCurrent = mouseXRef = event.button.x;
                mouseYCurrent = mouseYRef = event.button.y;
            }
        break;
        case SDL_MOUSEBUTTONUP:
            if (event.button.button == SDL_BUTTON_LEFT)
            {
                mouseGrabbed = 0;
            }
        break;
        case SDL_MOUSEMOTION:
        {
            mouseXCurrent = event.motion.x;
            mouseYCurrent = event.motion.y;
        }
        break;
        } // end switch
    } // end of message processing
    return done;
}

void controlView()
{
    double tmp[3];
    double tmpMatrix[9];
    double newOrientation[9];

    const double MOVEMENT_FACTOR = 0.5;
    const double TURN_FACTOR = 0.1;
    const double MOUSE_TURN_FACTOR = 0.03;
    // Tranlations
    if (movement.moveLeft)
    {
        tmp[0] = -camOrientation[0];
        tmp[1] = -camOrientation[3];
        tmp[2] = -camOrientation[6];
        ALG_scale(tmp, MOVEMENT_FACTOR);
        ALG_translate(camPosition, tmp);
    }
    if (movement.moveRight)
    {
        tmp[0] = camOrientation[0];
        tmp[1] = camOrientation[3];
        tmp[2] = camOrientation[6];
        ALG_scale(tmp, MOVEMENT_FACTOR);
        ALG_translate(camPosition, tmp);
    }
    if (movement.moveDown)
    {
        tmp[0] = -camOrientation[1];
        tmp[1] = -camOrientation[4];
        tmp[2] = -camOrientation[7];
        ALG_scale(tmp, MOVEMENT_FACTOR);
        ALG_translate(camPosition, tmp);
    }
    if (movement.moveUp)
    {
        tmp[0] = camOrientation[1];
        tmp[1] = camOrientation[4];
        tmp[2] = camOrientation[7];
        ALG_scale(tmp, MOVEMENT_FACTOR);
        ALG_translate(camPosition, tmp);
    }
    if (movement.moveForward)
    {
        tmp[0] = -camOrientation[2];
        tmp[1] = -camOrientation[5];
        tmp[2] = -camOrientation[8];
        ALG_scale(tmp, MOVEMENT_FACTOR);
        ALG_translate(camPosition, tmp);
    }
    if (movement.moveBackward)
    {
        tmp[0] = camOrientation[2];
        tmp[1] = camOrientation[5];
        tmp[2] = camOrientation[8];
        ALG_scale(tmp, MOVEMENT_FACTOR);
        ALG_translate(camPosition, tmp);
    }
    // Rotations
    if (movement.pitchDown)
    {
        tmp[0] = -camOrientation[0];
        tmp[1] = -camOrientation[3];
        tmp[2] = -camOrientation[6];
        ALG_createRotationMatrix(tmpMatrix, tmp, TURN_FACTOR);
        ALG_multiplyMatrix(newOrientation, tmpMatrix, camOrientation);
        memcpy(camOrientation, newOrientation, sizeof(newOrientation));
    }
    if (movement.pitchUp)
    {
        tmp[0] = camOrientation[0];
        tmp[1] = camOrientation[3];
        tmp[2] = camOrientation[6];
        ALG_createRotationMatrix(tmpMatrix, tmp, TURN_FACTOR);
        ALG_multiplyMatrix(newOrientation, tmpMatrix, camOrientation);
        memcpy(camOrientation, newOrientation, sizeof(newOrientation));
    }
    if (movement.turnRight)
    {
        tmp[0] = -camOrientation[1];
        tmp[1] = -camOrientation[4];
        tmp[2] = -camOrientation[7];
        ALG_createRotationMatrix(tmpMatrix, tmp, TURN_FACTOR);
        ALG_multiplyMatrix(newOrientation, tmpMatrix, camOrientation);
        memcpy(camOrientation, newOrientation, sizeof(newOrientation));
    }
    if (movement.turnLeft)
    {
        tmp[0] = camOrientation[1];
        tmp[1] = camOrientation[4];
        tmp[2] = camOrientation[7];
        ALG_createRotationMatrix(tmpMatrix, tmp, TURN_FACTOR);
        ALG_multiplyMatrix(newOrientation, tmpMatrix, camOrientation);
        memcpy(camOrientation, newOrientation, sizeof(newOrientation));
    }
    if (movement.bankRight)
    {
        tmp[0] = -camOrientation[2];
        tmp[1] = -camOrientation[5];
        tmp[2] = -camOrientation[8];
        ALG_createRotationMatrix(tmpMatrix, tmp, TURN_FACTOR);
        ALG_multiplyMatrix(newOrientation, tmpMatrix, camOrientation);
        memcpy(camOrientation, newOrientation, sizeof(newOrientation));
    }
    if (movement.bankLeft)
    {
        tmp[0] = camOrientation[2];
        tmp[1] = camOrientation[5];
        tmp[2] = camOrientation[8];
        ALG_createRotationMatrix(tmpMatrix, tmp, TURN_FACTOR);
        ALG_multiplyMatrix(newOrientation, tmpMatrix, camOrientation);
        memcpy(camOrientation, newOrientation, sizeof(newOrientation));
    }

    // Mouse things
    if (mouseGrabbed)
    {
        double dx = mouseXCurrent - mouseXRef;
        double dy = mouseYCurrent - mouseYRef;

        tmp[0] = -camOrientation[1] * MOUSE_TURN_FACTOR * dx;
        tmp[1] = -camOrientation[4] * MOUSE_TURN_FACTOR * dx;
        tmp[2] = -camOrientation[7] * MOUSE_TURN_FACTOR * dx;
        ALG_createRotationMatrix(tmpMatrix, tmp, TURN_FACTOR);
        ALG_multiplyMatrix(newOrientation, tmpMatrix, camOrientation);
        memcpy(camOrientation, newOrientation, sizeof(newOrientation));

        tmp[0] = -camOrientation[0] * MOUSE_TURN_FACTOR * dy;
        tmp[1] = -camOrientation[3] * MOUSE_TURN_FACTOR * dy;
        tmp[2] = -camOrientation[6] * MOUSE_TURN_FACTOR * dy;
        ALG_createRotationMatrix(tmpMatrix, tmp, TURN_FACTOR);
        ALG_multiplyMatrix(newOrientation, tmpMatrix, camOrientation);
        memcpy(camOrientation, newOrientation, sizeof(newOrientation));

        mouseXRef = mouseXCurrent;
        mouseYRef = mouseYCurrent;
    }
    // orthogonalize and normalize the orientation matrix
    ALG_transposeMatrix(camOrientation); //< Turn the column vector of the matrix into row vectors.
    // orthogonalize
    ALG_crossProduct(&camOrientation[6], &camOrientation[0], &camOrientation[3]); //< Z will be prependicular to X and Y
    ALG_crossProduct(&camOrientation[3], &camOrientation[6], &camOrientation[0]); //< Y will be prependicular to X and Z.
    // The 3 vectors are now prependicular to each other.
    // ensure unit length
    ALG_normalizeVector(&camOrientation[0]);
    ALG_normalizeVector(&camOrientation[3]);
    ALG_normalizeVector(&camOrientation[6]);
     // restore column vectors
    ALG_transposeMatrix(camOrientation);

}

void applyImpulse(
    DYN_Context *context,
    DYN_Body *body,
    const double *pointOfForce,
    const double *impulseVector
);

void createDisplayLists()
{
    cubeModel = glGenLists(1);
    // Load cube model
    glNewList(cubeModel, GL_COMPILE);
        glBegin(GL_QUADS);
        {
            glColor3f(1,1,1);
            // Draw top face
            glNormal3f(0, 1, 0);
            glVertex3f(1, 1, 1);
            glVertex3f(1, 1, -1);
            glVertex3f(-1, 1, -1);
            glVertex3f(-1, 1, 1);
            // Draw bottom face
            glNormal3f(0, -1, 0);
            glVertex3f(1, -1, 1);
            glVertex3f(-1, -1, 1);
            glVertex3f(-1, -1, -1);
            glVertex3f(1, -1, -1);
            // Draw left side face
            glNormal3f(-1, 0, 0);
            glVertex3f(-1, 1, 1);
            glVertex3f(-1, 1, -1);
            glVertex3f(-1, -1, -1);
            glVertex3f(-1, -1, 1);
            // Draw right side face
            glNormal3f(1, 0, 0);
            glVertex3f(1, 1, 1);
            glVertex3f(1, -1, 1);
            glVertex3f(1, -1, -1);
            glVertex3f(1, 1, -1);
            // Draw front side face
            glNormal3f(0, 0, -1);
            glVertex3f(1, 1, -1);
            glVertex3f(1, -1, -1);
            glVertex3f(-1, -1, -1);
            glVertex3f(-1, 1, -1);
            // Draw back side face
            glNormal3f(0, 0, 1);
            glVertex3f(1, 1, 1);
            glVertex3f(-1, 1, 1);
            glVertex3f(-1, -1, 1);
            glVertex3f(1, -1, 1);
        }
        glEnd();
    glEndList();

}

int main ( int argc, char** argv )
{
    // Initialize world.

    DYN_initialize(&world, 0.04);
    {
        DYN_Body body;
        DYN_BodyStaticAttributes attributes;

        double nullVector[3] = {0, 0, 0};
        double identity[9] =
        {
            1, 0, 0,
            0, 1, 0,
            0, 0, 1
        };

//        double impulse[3] = {100, 0, 0};
//        double pointOfImpulse[3] = {0, 1, 1};

        attributes.shape = DYN_CUBOID;
        attributes.cuboidAttributes.width = 1;
        attributes.cuboidAttributes.height = 2;
        attributes.cuboidAttributes.depth = 3;

        DYN_calculateMass(&attributes, 3);

        memcpy(body.position, nullVector, sizeof(nullVector));
        memcpy(body.velocity, nullVector, sizeof(nullVector));
        memcpy(body.angularVelocity, nullVector, sizeof(nullVector));
        memcpy(body.orientation, identity, sizeof(identity));

        DYN_addBody(&world, &body, &attributes);

        //applyImpulse(&world, &world.bodies[0], pointOfImpulse, impulse);

        body.position[0] = 5;
        body.position[1] = 5;
        body.position[2] = 5;

        DYN_addBody(&world, &body, &attributes);
    }

    // initialize SDL video
    if ( SDL_Init( SDL_INIT_VIDEO ) < 0 )
    {
        printf( "Unable to init SDL: %s\n", SDL_GetError() );
        return 1;
    }

    // make sure SDL cleans up before exit
    atexit(SDL_Quit);

    // Setting OpenGL  attributes

    SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE, 16 );
    SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );

    // create a new window
    SDL_Surface* screen = SDL_SetVideoMode(1024, 768, 16,
                                           SDL_HWSURFACE | SDL_OPENGL);

    // Create display lists

    createDisplayLists();

    // Initialize modes

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);

    // Initialize gl matrixes

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, 4.0/3.0, 1, 100000.0);

    // Initlalize GL lights

    {
        GLfloat ambient[4] = {0.1, 0.1, 0.1, 1};

        GLfloat diffuseColor[4] = {1, 1, 1, 1};
        GLfloat specularColor[4] = {0,0,0,0};

        glEnable(GL_LIGHTING);
        glEnable(GL_CULL_FACE);
        glEnable(GL_LIGHT0);

        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

        glMatrixMode(GL_MODELVIEW);

        glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseColor);
        glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
        glLightfv(GL_LIGHT0, GL_SPECULAR, specularColor);
        glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0);

    }


    if ( !screen )
    {
        printf("Unable to set 640x480 video: %s\n", SDL_GetError());
        return 1;
    }

    // program main loop
    while (!handleEvents())
    {
        // message processing loop

        DYN_stepWorld(&world);
        controlView();
        draw();
        usleep(40000);

        // finally, update the screen
        SDL_GL_SwapBuffers();
    } // end main loop

    // all is well ;)
    printf("Exited cleanly\n");
    return 0;
}
