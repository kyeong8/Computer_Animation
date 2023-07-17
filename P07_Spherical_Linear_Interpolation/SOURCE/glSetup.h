#ifndef __GL_SETUP_H_
#define __GL_SETUP_H_

#include <GL/glew.h>				// OpenGL Extension Wrangler Library
#include <GLFW/glfw3.h>

extern bool		fullScreen;			// Start window with full screen
extern bool		noMenuBar;			// with no menu bar

extern float	screenScale;		// portion of the screen when not using full screen
extern int		screenW, screenH;	// screenScale protion of the screen
extern int		windowW, windowH;	// Framebuffer size
extern float	aspect;				// Aspect ration = windowW/windowH
extern float	dpiScaling;			// DPI scaling for HIDPI

extern int		vsync;				// Vertical sync on/off

extern bool		perspectiveView;			// Perspective or orthographic viewing
extern float	fovy;				// Field of view in the y direction
extern float	nearDist;			// Near plane
extern float	farDist;			// Far plane

GLFWwindow* initializeOpenGL(int argc, char* argv[], GLfloat bg[4], bool modern = false);
void		reshape(GLFWwindow* window, int w, int h);
void		setupProjectionMatrix();

void		drawAxes(float l, float w);

#endif	// __GL_SETUP_H_
