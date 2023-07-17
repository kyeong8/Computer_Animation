
#include "glSetup.h"

#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>
using namespace std;

void init();
void quit();
void initializeParticleSystem();
void update(float delta_t);
void solveODE(float f);
void collisionHandling();
void keyboard(GLFWwindow* window, int key, int code, int action, int mods);
void render(GLFWwindow* window);
void setupLight();
void setupMaterial();
void drawSphere(float radius, const Vector3f& color, int N);

// Play configuration
bool	pause = true;
float	timeStep = 1.0f / 120;		// 120fps
int		N_SUBSTEPS = 1;				// Time stepping: h = timeStemp / N_SUBSTEPS

// Light configuration
Vector4f	light(0.0f, 0.0f, 5.0f, 1.0f);		// Light position

// Global coordinate frame
float	AXIS_LENGTH = 3;
float	AXIS_LINE_WIDTH = 2;

// Colors
GLfloat bgColor[4] = { 1, 1, 1, 1 };

// Sphere
GLUquadricObj* sphere = NULL;

// Particles
const int	nParticles = 100;
Vector3f	x[nParticles];	// Particle position
Vector3f	v[nParticles];	// Particle velocity
Vector3f	c[nParticles];	// Particle color

// Geometry and mass
float	radius = 0.02f;			// 2cm
float	m = 0.01f;				// 10kg

// External force
float		useGravity = true;
Vector3f	gravity(0, -9.8f, 0);	// Gravity -9.8m/s^2

// Collision
float	k_r = 0.9f;					// Coefficient of restitution
float	epsilon = 1.0E-4f;

const int	nWalls = 4;
Vector3f	wallP[nWalls];		// Points in the walls
Vector3f	wallN[nWalls];		// Normal vectors of the walls

// Method
enum IntegrationMethod
{
	EULER = 1,
	MODIFIED_EULER,
};	IntegrationMethod intMethod = MODIFIED_EULER;

int 
main(int argc, char* argv[])
{
	// Vertical sync on
	vsync = 1;

	// Orthographics viewing
	perspectiveView = false;

	// Initialize the OpenGL system
	GLFWwindow* window = initializeOpenGL(argc, argv, bgColor);
	if (window == NULL) return -1;

	// Callbacks
	glfwSetKeyCallback(window, keyboard);

	// Depth Test
	glDisable(GL_DEPTH_TEST);

	// Normal vectors are normalized after transformation
	glEnable(GL_NORMALIZE);

	// Back face culling
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);

	// Viewport and perspective setting
	reshape(window, windowW, windowH);

	// Initialization - Main loop - Fianlization
	init();
	initializeParticleSystem();

	// Main loop
	float	previous = (float)glfwGetTime();
	float	elapsed = 0;
	while (!glfwWindowShouldClose(window))
	{
		glfwPollEvents();				// Events

		// Time passed during a single loop
		float	now = (float)glfwGetTime();
		float	delta = now - previous;
		previous = now;

		// Time passed after the previous frame
		elapsed += delta;

		// Deal with the currnet frame
		if (elapsed > timeStep)
		{
			if (!pause) update(elapsed);

			elapsed = 0;	// Reset the elapsed time
		}

		render(window);				// Draw one frame
		glfwSwapBuffers(window); 	// Swap buffers
	}

	// Finalization
	quit();

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

void
initializeParticleSystem()
{
	// To generate the same random values at each execution
	srand(0);

	// Initialize particles with random positions and velocities
	for (int i = 0; i < nParticles; i++)
	{
		// Position
		x[i] = Vector3f::Random();

		x[i][0] *= aspect;	// x-axis
		x[i][2] = 0;		// z-axis

		// Velocity
		v[i] = Vector3f::Random();
		v[i][2] = 0;		// z-axis

		// Color
		c[i] = VectorXf::Random();
		c[i] = (c[i] + Vector3f(1, 1, 1)) / 2.0;
		c[i].normalize();
	}

	// Normal vectors of the 4 surrounding walls
	wallN[0] = Vector3f( 1.0f, 0, 0);	// Left
	wallN[1] = Vector3f(-1.0f, 0, 0);	// Left
	wallN[2] = Vector3f(0,  1.0f, 0);	// Left
	wallN[3] = Vector3f(0, -1.0f, 0);	// Left
	
	for (int i = 0; i < nWalls; i++)
		wallN[i].normalize();

	// Collision handling
	collisionHandling();
}

void
update(float delta_t)
{
	// Solve ordinary differential equation
	for (int i = 0; i < N_SUBSTEPS; i++)
	{
		float	h = delta_t / N_SUBSTEPS;	// Time step
		solveODE(h);
	}
}

void
solveODE(float h)
{
	for (int i = 0; i < nParticles; i++)
	{
		// External force
		Vector3f	f(0, 0, 0);

		// Gravity
		if (useGravity) f += m * gravity;

		// Time stepping
		switch (intMethod)
		{
		case EULER:
			x[i] += h * v[i];
			v[i] += h * f / m;
			break;
		
		case MODIFIED_EULER:
			v[i] += h * f / m;
			x[i] += h * v[i];
			break;
		}
	}

	// Collision handling
	collisionHandling();
}

void
collisionHandling()
{
	// Points of the 4 surrounding walls: It can changes when the aspect ratio changes.
	wallP[0] = Vector3f(-1.0f * aspect, 0, 0);	// Left
	wallP[1] = Vector3f( 1.0f * aspect, 0, 0);	// Right
	wallP[2] = Vector3f(0, -1.0f, 0);			// Bottom
	wallP[3] = Vector3f(0, 1.0f, 0);			// Top

	// Collision wrt the walls
	for (int i = 0; i < nParticles; i++)
	{
		for (int j = 0; j < nWalls; j++)
		{
			float	d_n = wallN[j].dot(x[i] - wallP[j]);	// Distance to the wall
			if (d_n < radius + epsilon)
			{
				// Position correction
				x[i] += (radius + epsilon - d_n) * wallN[j];

				// Velocity in the normal direction
				float	v_n = wallN[j].dot(v[i]);

				if (v_n < 0)	// Heading into the wall
				{
					// Velocity correction: v[i] - v_n = v_t
					v[i] -= (1 + k_r) * v_n * wallN[j];
				}
			}
		}
	}
}

void 
init()
{
	// Prepare quadric shapes
	sphere = gluNewQuadric();
	gluQuadricDrawStyle(sphere, GLU_FILL);
	gluQuadricNormals(sphere, GLU_SMOOTH);
	gluQuadricOrientation(sphere, GLU_OUTSIDE);
	gluQuadricTexture(sphere, GL_FALSE);

	// Keyboard and mouse
	cout << "Keyboard input: space for play/pause" << endl;
	cout << "Keyboard Input: g for gravity on/off" << endl;
	cout << "Keyboard Input: i for the system initialization" << endl;
	cout << "Keyboard Input: e for the Euler integration" << endl;
	cout << "Keyboard Input: m for the modified Euler integration" << endl;
	cout << "Keyboard Input: [1:9] for the # of sub-time steps" << endl;
}

void
quit()
{
	// Delete quadric shapes
	gluDeleteQuadric(sphere);
}

// Light
void
setupLight()
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	GLfloat ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat diffuse[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat specular[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light.data());
}

// Material
void
setupMaterial()
{
	// Material
	GLfloat mat_ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat mat_specular[4] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat mat_shininess = 128;

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
}

void
setDiffuseColor(const Vector3f& color)
{
	GLfloat mat_diffuse[4] = { color[0], color[1], color[2], 1 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
}

// Draw a sphere after setting up its material
void
drawSphere(float radius, const Vector3f& color, int N)
{
	// Material
	setDiffuseColor(color);

	// Sphere using GLU quadrics
	gluSphere(sphere, radius, N, N);
}

void 
render(GLFWwindow* window)
{
	// Background color
	glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Lighting
	setupLight();

	// Material
	setupMaterial();

	// Particles
	for (int i = 0; i < nParticles; i++)
	{
		glPushMatrix();
		glTranslatef(x[i][0], x[i][1], x[i][2]);
		drawSphere(radius, c[i], 20);
		glPopMatrix();
	}
}

void 
keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT)
	{
		switch (key)
		{
			// Quit
		case GLFW_KEY_Q:
		case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GL_TRUE);	break;

			// Sub-time steps
		case GLFW_KEY_1: N_SUBSTEPS = 1; break;
		case GLFW_KEY_2: N_SUBSTEPS = 2; break;
		case GLFW_KEY_3: N_SUBSTEPS = 3; break;
		case GLFW_KEY_4: N_SUBSTEPS = 4; break;
		case GLFW_KEY_5: N_SUBSTEPS = 5; break;
		case GLFW_KEY_6: N_SUBSTEPS = 6; break;
		case GLFW_KEY_7: N_SUBSTEPS = 7; break;
		case GLFW_KEY_8: N_SUBSTEPS = 8; break;
		case GLFW_KEY_9: N_SUBSTEPS = 9; break;

			// Controls
		case GLFW_KEY_SPACE:	pause = !pause;	break;				// Play on/off
		case GLFW_KEY_G:		useGravity = !useGravity;	break;	// Gravity on/off
		case GLFW_KEY_I:		initializeParticleSystem();	break;	// Initialization
		case GLFW_KEY_E:		intMethod = EULER;			break;
		case GLFW_KEY_M:		intMethod = MODIFIED_EULER;	break;
		}
	}
}
