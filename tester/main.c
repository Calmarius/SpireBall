#ifdef __APPLE__
#include <SDL/SDL.h>
#else
#include <SDL.h>
#endif
#include <unistd.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>

void draw()
{
    // Setting camera

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(5, 5, 5, 0, 0, 0, 0, 1, 0);

    // Drawing
    glClearColor(0, 0, 0.5f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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
                // exit if ESCAPE is pressed
                if (event.key.keysym.sym == SDLK_ESCAPE)
                    done = 1;
                break;
            }
        } // end switch
    } // end of message processing
    return done;

}

int main ( int argc, char** argv )
{
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

    // Initialize modes

    glEnable(GL_DEPTH_TEST);

    // Initialize gl matrixes

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, 4.0/3.0, 1, 100000.0);

    if ( !screen )
    {
        printf("Unable to set 640x480 video: %s\n", SDL_GetError());
        return 1;
    }

    // program main loop
    while (!handleEvents())
    {
        // message processing loop

        draw();
        usleep(100000);

        // finally, update the screen
        SDL_GL_SwapBuffers();
    } // end main loop

    // all is well ;)
    printf("Exited cleanly\n");
    return 0;
}
