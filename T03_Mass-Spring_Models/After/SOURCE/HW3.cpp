#include "glSetup.h"

#ifdef  _WIN32
#define _USE_MATH_DEFINES	// To include the definition of M_PI in math.h
#endif

#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>
#include <vector>
using namespace std;

void	init();
void	quit();
void	initializeParticle();
void	update(float delta_t);
void	solveODE(float h);
void	collisionHandling();
void	render(GLFWwindow* window);
void	setupLight();
void	setupMaterial();
void	drawSphere(float radius, const Vector3f& color, int N);
void	reshape(GLFWwindow* window, int w, int h);
void	keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);
void	mouseButton(GLFWwindow* window, int button, int action, int mods);
void	mouseMove(GLFWwindow* window, double x, double y);

// Play configuration
bool	pause = true;
float	timeStep = 1.0f / 120;		// 120fps
int		N_SUBSTEPS = 1;				// Time stepping: h = timeStemp / N_SUBSTEPS

// Light configuration
Vector4f	light(0.0f, 0.0f, 5.0f, 1.0f);		// Light position

// Colors
GLfloat bgColor[4] = { 1, 1, 1, 1 };

// Sphere
GLUquadricObj* sphere = NULL;

int			nParticles = 0;
vector<vector<float>>	x;		// Particle position
vector<vector<float>>	v;		// Particle velocity

vector<vector<float>>	memX;		// Particle velocity
vector<vector<float>>	memV;		// Particle velocity

// Connectivity
int		nEdges = 0;
vector<int>		e1;				// One end of the edge
vector<int>		e2;				// The other end of the edge
vector<float>	l;				// Rest lenghts between particles
vector<float>	k;				// Spring constants
float			k0 = 1.0f;		// Global spring constant

vector<bool>	constrained;

bool	usePointDamping = false;
float	damping = 0.01f;

bool	useSpringDamping = false;

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

float		range = 0.02;
vector<int>	select = { -1, -1 };
int			countSelect = 0;
int			picked = -1;
bool		flag = false;

// Picking
enum PickMode {
	NONE = 0, CREATE = 1, ATTACH = 2, DRAG = 3, COMPLETE = 4, NAIL = 5,
};	PickMode pickMode = NONE;

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
	glfwSetMouseButtonCallback(window, mouseButton);
	glfwSetCursorPosCallback(window, mouseMove);

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
	initializeParticle();

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

void
rebuildSpringK(bool printFlag)
{
	if (printFlag)
	{
		cout << "Spring constant = " << k0 << endl;
		cout << endl;
	}

	// Spring constants
	for (int i = 0; i < nEdges; i++)
		k[i] = k0 / l[i];	// Inversely proportion to the spring length
}

void
initializeParticle()
{
	nParticles = 0;
	x.clear();
	v.clear();

	nEdges = 0;
	e1.clear();				// One end of the edge
	e2.clear();				// The other end of the edge
	l.clear();				// Rest lenghts between particles
	k.clear();				// Spring constants
	constrained.clear();

	select.assign(2, -1);
	countSelect = 0;
}

void
backupPV()
{
	if (x.size() > 0)
	{
		memX.assign(x.begin(), x.end());
		memV.assign(v.begin(), v.end());
	}
}

void
undoPV()
{
	if (memX.size() > 0)
	{
		x.assign(memX.begin(), memX.end());
		v.assign(memV.begin(), memV.end());
	}
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

VectorXf
toEigenVector(const vector<float>& v) {
	VectorXf ev(v.size());
	for (int i = 0; i < v.size(); i++) {
		ev(i) = v[i];
	}
	return ev;
}

void
solveODE(float h)
{
	// Total force
	Vector3f*	f = new Vector3f[nParticles];
	Vector3f eigenVector;

	for (int i = 0; i < nParticles; i++)
	{
		// Initialization
		f[i].setZero();

		// Gravity
		if (useGravity) f[i] += m * gravity;

		// Point damping
		if (usePointDamping)	f[i] -= damping * toEigenVector(v[i]);
	}

	// Spring force
	for (int i = 0; i < nEdges; i++)
	{
		Vector3f	v_i = toEigenVector(x[e1[i]]) - toEigenVector(x[e2[i]]);
		float		L_i = v_i.norm();			// Current length
		Vector3f	f_i = k[i] * (L_i - l[i]) * (v_i / L_i);

		f[e2[i]] += f_i;
		f[e1[i]] -= f_i;

		if (useSpringDamping)
		{
			Vector3f	d_ij = v_i / L_i;
			Vector3f	v_ij = toEigenVector(v[e1[i]]) - toEigenVector(v[e2[i]]);

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
			eigenVector = toEigenVector(x[i]) + h * toEigenVector(v[i]);
			x[i].assign(eigenVector.data(), eigenVector.data() + eigenVector.size());

			eigenVector = toEigenVector(v[i]) + h * f[i] / m;
			v[i].assign(eigenVector.data(), eigenVector.data() + eigenVector.size());
			break;

		case MODIFIED_EULER:
			eigenVector = toEigenVector(v[i]) + h * f[i] / m;
			v[i].assign(eigenVector.data(), eigenVector.data() + eigenVector.size());

			eigenVector = toEigenVector(x[i]) + h * toEigenVector(v[i]);
			x[i].assign(eigenVector.data(), eigenVector.data() + eigenVector.size());
			break;
		}
	}

	// Collision handling
	collisionHandling();
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
	cout << "Keyboard Input: c for Create Particles" << endl;
	cout << "Keyboard Input: a for Attach Springs" << endl;
	cout << "Keyboard Input: d for Select/drag Particles" << endl;
	cout << "Keyboard Input: n for Nail Particles" << endl;
	cout << "Keyboard Input: g for gravity on/off" << endl;
	cout << "Keyboard Input: e for the Euler integration" << endl;
	cout << "Keyboard Input: m for the modified Euler integration" << endl;
	cout << "Keyboard Input: p for point damping on/off" << endl;
	cout << "Keyboard Input: s for spring damping on/off" << endl;
	cout << "Keyboard Input: i for Initialize all Particle" << endl;
	cout << "Keyboard Input: r for Remove all nail constraints" << endl;
	cout << "Keyboard Input: b for Backup all Particle" << endl;
	cout << "Keyboard Input: u for Undo all Particle" << endl;
	cout << "Keyboard Input: [1:9] for the # of sub-time steps" << endl;
	cout << "Keyboard Input: up/down to increase/decrease the spring constant" << endl;
	cout << "Keyboard Input: left/right to increase/decrease the damping constant" << endl;
	cout << endl;

	// Normal vectors of the 4 surrounding walls
	wallN[0] = Vector3f(1.0f, 0, 0);	// Left
	wallN[1] = Vector3f(-1.0f, 0, 0);	// Right
	wallN[2] = Vector3f(0, 1.0f, 0);	// Up
	wallN[3] = Vector3f(0, -1.0f, 0);	// Down

	for (int i = 0; i < nWalls; i++)
		wallN[i].normalize();

	// Collision handling
	collisionHandling();
	
	rebuildSpringK(true);
}

void
collisionHandling()
{
	// Points of the 4 surrounding walls: It can changes when the aspect ratio changes.
	wallP[0] = Vector3f(-1.0f * aspect, 0, 0);	// Left
	wallP[1] = Vector3f(1.0f * aspect, 0, 0);	// Right
	wallP[2] = Vector3f(0, -1.0f, 0);			// Bottom
	wallP[3] = Vector3f(0, 1.0f, 0);			// Top

	// Collision wrt the walls
	for (int i = 0; i < nParticles; i++)
	{
		for (int j = 0; j < nWalls; j++)
		{
			float	d_n = wallN[j].dot(toEigenVector(x[i]) - wallP[j]);	// Distance to the wall
			if (d_n < radius + epsilon)
			{
				// Position correction
				Vector3f eigenVector = toEigenVector(x[i]) + (radius + epsilon - d_n) * wallN[j];
				x[i].assign(eigenVector.data(), eigenVector.data() + eigenVector.size());

				// Velocity in the normal direction
				float	v_n = wallN[j].dot(toEigenVector(v[i]));

				if (v_n < 0)	// Heading into the wall
				{
					// Velocity correction: v[i] - v_n = v_t
					Vector3f eigenVector = toEigenVector(v[i]) - (1 + k_r) * v_n * wallN[j];
					v[i].assign(eigenVector.data(), eigenVector.data() + eigenVector.size());
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
		else if (find(select.begin(), select.end(), i) != select.end())
			drawSphere(radius, Vector3f(1, 0, 0), 20);
		else if (picked == i)
			drawSphere(radius, Vector3f(1, 0, 0), 20);
		else
			drawSphere(radius, Vector3f(0, 1, 0), 20);
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

int
findNearestPoint(float xw, float yw)
{
	float nearest = range;
	int name = -1;

	for (int i = 0; i < nParticles; i++)
	{
		Vector3f p1(xw, yw, 0);
		Vector3f p2 = toEigenVector(x[i]);

		Vector3f dist = p2 - p1;
		float distance = dist.norm();

		if (nearest > distance)
		{
			nearest = distance;
			name = i;
		}
	}

	if (name > -1)	return name;

	return -1;
}

void
createParticle(float xw, float yw)
{
	// Initialize particles with cursor position and velocitie
	x.push_back({ xw, yw, 0.0 });
	v.push_back({ 0.0, 0.0, 0.0 });

	constrained.push_back(false);

	nParticles += 1;
}

void attachParticle(float xw, float yw)
{
	int point = findNearestPoint(xw, yw);

	if (point > -1 && countSelect < 2)
	{
		if (select[countSelect - 1] != point)
		{
			select[countSelect] = point;
			countSelect += 1;
		}
	}

	if (point > -1 && countSelect == 2)
	{
		e1.push_back(select[0]);
		e2.push_back(select[1]);

		l.push_back((toEigenVector(x[e1[nEdges]]) - toEigenVector(x[e2[nEdges]])).norm());
		k.push_back(k0 / l[nEdges]);

		nEdges += 1;

		select.assign(2, -1);
		countSelect = 0;
	}
}

void
keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if ((action == GLFW_PRESS || action == GLFW_REPEAT) && !(mods & GLFW_MOD_CONTROL))
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

			// Method
		case GLFW_KEY_E:
			intMethod = EULER;
			cout << "Method is Euler" << endl;	cout << endl; break;

		case GLFW_KEY_M:
			intMethod = MODIFIED_EULER;
			cout << "Method is Modified Euler" << endl; cout << endl; break;

			// Operate
		case GLFW_KEY_C:	cout << "Change CREATE Mode\n" << endl; pickMode = CREATE;	select.assign(2, -1);	countSelect = 0; break;

		case GLFW_KEY_A:	cout << "Change ATTACH Mode\n" << endl; pickMode = ATTACH;	select.assign(2, -1);	countSelect = 0; break;

		case GLFW_KEY_D:	cout << "Change DRAG Mode\n" << endl; pickMode = COMPLETE;	select.assign(2, -1);	countSelect = 0; break;

		case GLFW_KEY_N:	cout << "Change NAIL Mode\n" << endl; pickMode = NAIL;	select.assign(2, -1);	countSelect = 0; break;

		case GLFW_KEY_I:    cout << "Initialize Particle\n" << endl; initializeParticle(); break;

		case GLFW_KEY_B: cout << "Backup all Particle\n" << endl; backupPV(); flag = true; break;

		case GLFW_KEY_U: cout << "Undo all Particle\n" << endl; undoPV(); flag = false; break;
		
		case GLFW_KEY_R:    cout << "Remove all nail constraints\n" << endl; constrained.assign(nParticles, false); break;

			// Spring constants
		case GLFW_KEY_UP:	k0 = min(k0 + 0.1f, 10.0f);		rebuildSpringK(true);	break;
		case GLFW_KEY_DOWN:	k0 = max(k0 - 0.1f, 0.1f);		rebuildSpringK(true);	break;

			// Damping constant
		case GLFW_KEY_P:
			usePointDamping = !usePointDamping;
			if (usePointDamping) cout << "usePointDamping is " << "True" << endl;
			else cout << "usePointDamping is " << "False" << endl;
			cout << endl; break;
		case GLFW_KEY_S:
			useSpringDamping = !useSpringDamping;
			if (useSpringDamping) cout << "useSpringDamping is " << "True" << endl;
			else cout << "useSpringDamping is " << "False" << endl;
			cout << endl; break;

		case GLFW_KEY_LEFT:		damping = max(damping - 0.01f, 0.0f);	cout << "Damping coeff = " << damping << endl;	cout << endl; break;
		case GLFW_KEY_RIGHT:	damping = min(damping + 0.01f, 1.0f);	cout << "Damping coeff = " << damping << endl;	cout << endl; break;

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

	if ((action == GLFW_PRESS || action == GLFW_REPEAT) && (mods & GLFW_MOD_CONTROL))
	{
		switch (key)
		{
			// Navigation
		case GLFW_KEY_UP:		 range += 0.1f; cout << "range is " << range << endl; 	break;
		case GLFW_KEY_DOWN:		 range -= 0.1f; cout << "range is " << range << endl; 	break;

		case GLFW_KEY_SPACE:
			cout << "Particle Count is " << nParticles << endl;
			for (int i = 0; i < nParticles; i++)
			{
				for (int j = 0; j < 3; j++)	
					cout << x[i][j] << endl;
				cout << endl;
			}
		}
	}
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
screen2world(float xs, float ys, float& xw, float& yw)
{
	// In the world space. See reshape() in glSetup.cpp
	float	aspect = (float)screenW / screenH;
	xw = 2.0f * (xs / (screenW - 1) - 0.5f) * aspect;
	yw = -2.0f * (ys / (screenH - 1) - 0.5f);
}


void
mouseButton(GLFWwindow* window, int button, int action, int mods)
{
	// Mouse cursor position in the screen coordinate system
	double	xs, ys;
	glfwGetCursorPos(window, &xs, &ys);

	int nail = -1;

	float xw, yw;
	screen2world(xs * dpiScaling, ys * dpiScaling, xw, yw);

	if (action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_LEFT)
	{
		//cout << "Create" << endl;

		if (pickMode == COMPLETE)
			pickMode = DRAG;

		if (pickMode == CREATE)	createParticle(xw, yw);
		else if (pickMode == ATTACH) attachParticle(xw, yw);
		else if (pickMode == DRAG)
		{
			picked = findNearestPoint(xw, yw);
			if (picked > -1)
			{
				select[0] = picked;
				createParticle(xw, yw);
				constrained[nParticles - 1] = true;
				select[1] = nParticles - 1;
				picked = nParticles - 1;

				e1.push_back(select[0]);
				e2.push_back(select[1]);

				l.push_back((toEigenVector(x[e1[nEdges]]) - toEigenVector(x[e2[nEdges]])).norm());
				k.push_back(k0 / l[nEdges]);

				nEdges += 1;
			}
		}
		else if (pickMode == NAIL)
		{
			nail = findNearestPoint(xw, yw);
			if (nail > -1) constrained[nail] = true;
		}
	}

	if (action == GLFW_RELEASE && button == GLFW_MOUSE_BUTTON_LEFT)
	{
		picked = -1;
		if (pickMode == DRAG)
		{
			if (select[0] > -1)
			{
				pickMode = COMPLETE;
				select.assign(2, -1);
				countSelect = 0;
				x.pop_back();
				v.pop_back();
				constrained.pop_back();
				nParticles -= 1;

				e1.pop_back();
				e2.pop_back();

				l.pop_back();
				k.pop_back();

				nEdges -= 1;
			}
		}
	}
}

void
mouseMove(GLFWwindow* window, double xs, double ys)
{
	float xw, yw;

	// Update the end point while dragging
	if (pickMode == DRAG)
	{
		if (picked > -1)
		{
			screen2world(xs, ys, xw, yw);

			vector<float> res = x[select[1]];

			x[select[1]] = { xw, yw, 0.0 };
			x[select[0]][0] += x[select[1]][0] - res[0];
			x[select[0]][1] += x[select[1]][1] - res[1];
			x[select[0]][2] += x[select[1]][2] - res[2];

			//x[picked] = { xw, yw, 0.0 };

			//l[nEdges-1] = (toEigenVector(x[e1[nEdges-1]]) - toEigenVector(x[e2[nEdges-1]])).norm();
			//k[nEdges-1] = (k0 / l[nEdges-1]);
		}
	}
}