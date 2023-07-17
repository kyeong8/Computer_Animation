
#include "glSetup.h"

#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>
using namespace std;


void init();
void quit();
void initializeParticleSystem();
void update(float delta_t);
void solveODE(float h);
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
const int	nParticles = 4;
Vector3f	x[nParticles];	// Particle position
Vector3f	v[nParticles];	// Particle velocity

// Connectivity
const int	nEdges = 3;
int			e1[nEdges] = { 0, 1, 2 };	// One end of the edge
int			e2[nEdges] = { 2, 2, 3 };	// The other end of the edge
float		l[nEdges];					// Rest lenghts between particles
float		k[nEdges];					// Spring constants
float		k0 = 1.0f;					// Global spring constant

bool		useConst = true;			// Constraints
bool		constrained[nParticles];

bool	usePointDamping = true;
float	damping = 0.01f;

bool	useSpringDamping = true;

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

	// Vertical sync for 60fps
	glfwSwapInterval(1);

	// Callbacks
	glfwSetKeyCallback(window, keyboard);

	// Depth Test
	glEnable(GL_DEPTH_TEST);

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
rebuildSpringK()
{
	cout << "Spring constant = " << k0 << endl;
	cout << endl;

	// Spring constants
	for (int i = 0; i < nEdges; i++)
		k[i] = k0 / l[i];	// Inversely proportion to the spring length
}

void
initializeParticleSystem()
{
	// To generate the same random values at each execution
	srand(0);

	// Position
	x[0] = Vector3f(-0.3f, 0.5f, 0);
	x[1] = Vector3f( 0.3f, 0.5f, 0);
	x[2] = Vector3f( 0.0f, 0.2f, 0);

	float	l01 = (x[2] - x[0]).norm();
	x[3] = x[2] - Vector3f(0, l01, 0);

	// Initialize particles with random positions and velocities
	for (int i = 0; i < nParticles; i++)
	{
		// Zero initial velocity
		v[i].setZero();
		
		// Not constrained
		constrained[i] = false;
	}

	// Rest length
	for (int i = 0; i < nEdges; i++)
		l[i] = (x[e1[i]] - x[e2[i]]).norm();

	// Spring constants
	rebuildSpringK();

	// Normal vectors of the 4 surrounding walls
	wallN[0] = Vector3f( 1.0f, 0, 0);	// Left
	wallN[1] = Vector3f(-1.0f, 0, 0);	// Right
	wallN[2] = Vector3f(0,  1.0f, 0);	// Up
	wallN[3] = Vector3f(0, -1.0f, 0);	// Down
	
	for (int i = 0; i < nWalls; i++)
		wallN[i].normalize();

	// Collision handling
	collisionHandling();
}

void
quit()
{
	// Delete quadric shapes
	gluDeleteQuadric(sphere);
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
	// Total force
	Vector3f	f[nParticles];

	for (int i = 0; i < nParticles; i++)
	{
		// Initialization
		f[i].setZero();

		// Gravity
		if (useGravity) f[i] += m * gravity;

		// Point damping
		if (usePointDamping)	f[i] -= damping * v[i];
	}

	// Spring force
	for (int i = 0; i < nEdges; i++)
	{
		Vector3f	v_i = x[e1[i]] - x[e2[i]];
		float		L_i = v_i.norm();			// Current length
		Vector3f	f_i = k[i] * (L_i - l[i]) * (v_i / L_i);

		f[e2[i]] += f_i;
		f[e1[i]] -= f_i;

		if (useSpringDamping)
		{
			Vector3f	d_ij = v_i / L_i;
			Vector3f	v_ij = v[e1[i]] - v[e2[i]];

			Vector3f	f_ij = damping * v_ij.dot(d_ij) * d_ij;

			f[e1[i]] -= f_ij;
			f[e2[i]] += f_ij;
		}
	}

	for (int i = 0; i < nParticles; i++)
	{
		// Constraint
		if (constrained[i]) continue;

		// Time stepping
		switch (intMethod)
		{
		case EULER:
			x[i] += h * v[i];
			v[i] += h * f[i] / m;
			break;
		
		case MODIFIED_EULER:
			v[i] += h * f[i] / m;
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
		if (constrained[i]) drawSphere(radius, Vector3f(1, 1, 0), 20);
		else                drawSphere(radius, Vector3f(0, 1, 0), 20);
		glPopMatrix();
	}

	// Edges
	glLineWidth(7 * dpiScaling);
	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	for (int i = 0; i < nEdges; i++)
	{
		glVertex3fv(x[e1[i]].data());
		glVertex3fv(x[e2[i]].data());
	}
	glEnd();
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
		case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GL_TRUE); break;

			// Controls
		case GLFW_KEY_SPACE:	pause = !pause;	break;				// Play on/off
		case GLFW_KEY_G:	// Gravity on/off	
			useGravity = !useGravity;	
			if (useGravity) cout << "useGravity is " << "True" << endl;
			else cout << "useGravity is " << "False" << endl;
			cout << endl; break;

		case GLFW_KEY_I:		initializeParticleSystem();	break;
		case GLFW_KEY_E:		
			intMethod = EULER;
			cout << "Method is Euler" << endl;	cout << endl; break;
		case GLFW_KEY_M:		
			intMethod = MODIFIED_EULER;
			cout << "Method is Modified Euler" << endl; cout << endl; break;

		case GLFW_KEY_C:		
			useConst = !useConst;	// Consts ctrl on/off
			if (useConst) cout << "useConst is " << "True" << endl;
			else cout << "useConst is " << "False" << endl; 
			cout << endl; break;

			// Spring constants
		case GLFW_KEY_UP:	k0 = min(k0 + 0.1f, 10.0f);		rebuildSpringK();	break;
		case GLFW_KEY_DOWN:	k0 = max(k0 - 0.1f, 0.1f);		rebuildSpringK();	break;

			// Damping constant
		case GLFW_KEY_D:			
			usePointDamping = !usePointDamping;	
			if(usePointDamping) cout << "usePointDamping is " << "True" << endl;
			else cout << "usePointDamping is " << "False" << endl;
			cout << endl; break;
		case GLFW_KEY_S:		
			useSpringDamping = !useSpringDamping;
			if (useSpringDamping) cout << "useSpringDamping is " << "True" << endl;
			else cout << "useSpringDamping is " << "False" << endl;
			cout << endl; break;

		case GLFW_KEY_LEFT:	damping = max(damping - 0.01f, 0.0f);	cout << "Damping coeff = " << damping << endl;	cout << endl; break;
		case GLFW_KEY_RIGHT:	damping += 0.01f;	cout << "Damping coeff = " << damping << endl;	cout << endl; break;
		}

		if (useConst)
		{
			switch (key)
			{
				// Constraints
			case GLFW_KEY_1:	constrained[0] = !constrained[0];	break;
			case GLFW_KEY_2:	constrained[1] = !constrained[1];	break;
			case GLFW_KEY_3:	constrained[2] = !constrained[2];	break;
			case GLFW_KEY_4:	constrained[3] = !constrained[3];	break;
			}
		}
		else
		{
			switch (key)
			{
				// Sub-time steps
			case GLFW_KEY_1: N_SUBSTEPS = 1; cout << "N_SUB_SUBSTEPS = " << N_SUBSTEPS << endl; cout << endl; break;
			case GLFW_KEY_2: N_SUBSTEPS = 2; cout << "N_SUB_SUBSTEPS = " << N_SUBSTEPS << endl; cout << endl; break;
			case GLFW_KEY_3: N_SUBSTEPS = 3; cout << "N_SUB_SUBSTEPS = " << N_SUBSTEPS << endl; cout << endl; break;
			case GLFW_KEY_4: N_SUBSTEPS = 4; cout << "N_SUB_SUBSTEPS = " << N_SUBSTEPS << endl; cout << endl; break;
			case GLFW_KEY_5: N_SUBSTEPS = 5; cout << "N_SUB_SUBSTEPS = " << N_SUBSTEPS << endl; cout << endl; break;
			case GLFW_KEY_6: N_SUBSTEPS = 6; cout << "N_SUB_SUBSTEPS = " << N_SUBSTEPS << endl; cout << endl; break;
			case GLFW_KEY_7: N_SUBSTEPS = 7; cout << "N_SUB_SUBSTEPS = " << N_SUBSTEPS << endl; cout << endl; break;
			case GLFW_KEY_8: N_SUBSTEPS = 8; cout << "N_SUB_SUBSTEPS = " << N_SUBSTEPS << endl; cout << endl; break;
			case GLFW_KEY_9: N_SUBSTEPS = 9; cout << "N_SUB_SUBSTEPS = " << N_SUBSTEPS << endl; cout << endl; break;
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
	cout << "Keyboard Input: c for constraint specification" << endl;
	cout << "Keyboard Input: e for the Euler integration" << endl;
	cout << "Keyboard Input: m for the modified Euler integration" << endl;
	cout << "Keyboard Input: d for damping on/off" << endl;
	cout << "Keyboard Input: [1:9] for the # of sub-time steps" << endl;
	cout << "Keyboard Input: [1:4] for constraint specification" << endl;
	cout << "Keyboard Input: up/down to increase/decrease the spring constant" << endl;
	cout << "Keyboard Input: left/right to increase/decrease the damping constant" << endl;
	cout << endl;
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
