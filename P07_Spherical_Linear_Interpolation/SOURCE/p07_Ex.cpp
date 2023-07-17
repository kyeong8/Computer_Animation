#include "glSetup.h"
#include "mesh.h"

#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>
using namespace std;

#ifdef _WIN32
#define _USE_MATH_DEFINES	// To include the definition of M_PI in math.h
#endif

#include <math.h>

void	init(const char* filename);
void	setupLight();

void	update(float elapsed);
void	render(GLFWwindow* window);
void	reshape(GLFWwindow* window, int w, int h);
void	keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);

// Play configuration
bool	pause = false;

// Camera configuration
Vector3f	eye(9, 9, 9);
Vector3f	center(0, 0, 0);
Vector3f	up(0, 1, 0);

// Light configuration
Vector4f	light(5.0, 5.0, 0.0, 1);		// Light direction

// Global coordinate frame
float	AXIS_LENGTH = 3;
float	AXIS_LINE_WIDTH = 2;

// Colors
GLfloat bgColor[4] = { 1, 1, 1, 1 };

// Default mesh file name
const char* defaultMeshFileName = "m01_dinosaur.off";

// Mesh
MatrixXf	vertex;
MatrixXf	normal;
ArrayXXi	face;

// Time
float	timeStep = 1.0f / 120;		// 120fps
float	currTime = 0;

// Transformation
Matrix4f	T;		// Current orientation and position
Quaternionf	q1, q2;	// Two key orientations
Vector3f	p1, p2;	// Two key positions

// Interpolation duration
float	interval = 3;	// Seconds

// Interpolation method
int		method = 4;

const char* methodString[] = {
	"Linear interpolation of position",
	"Linear interpolation of rotation matrices",
	"Linear interpolation of unit quaternions with normalization",
	"Spherical linear interpolation using Eigen's slerp",
	"Spherical linear interpolation using Eigen's AngleAxis (Log)",
	"Spherical linear interpolation using Eigen's AngleAxis variation"
};

int
main(int argc, char* argv[])
{
	// vsync should be 0 for precise time stepping
	vsync = 0;

	// Filename for deformable body configuration
	const char* filename;
	if (argc >= 2)	filename = argv[1];
	else            filename = defaultMeshFileName;

	// Field of view of 85mm lens in degree
	fovy = 16.1f;

	// Initialize the OpenGL system
	GLFWwindow* window = initializeOpenGL(argc, argv, bgColor);
	if (window == NULL) return	-1;

	// Callbacks
	glfwSetKeyCallback(window, keyboard);
	
	// Depth test
	glEnable(GL_DEPTH_TEST);

	// Normal vectors are normalized after transformation.
	glEnable(GL_NORMALIZE);

	// Back face culling
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);

	// Viewport and perspective setting
	reshape(window, windowW, windowH);

	// Usage
	cout << endl;
	for (int i = 0; i < 6; i++)
		cout << "Keyboard Input: " << (i + 1) << " for " << methodString[i] << endl;
	cout << "Keyboard Input: " << "7 for Visual comparison for exercise" << endl;
	cout << endl;

	// Initialization - Main loop - Finalization
	init(filename);

	// Main loop
	float	previous = (float)glfwGetTime();
	float	elapsed = 0;
	while (!glfwWindowShouldClose(window))
	{
		// Time passed during a single loop
		float	now = (float)glfwGetTime();
		float	delta = now - previous;
		previous = now;
		
		// Time passed after the previous frame
		elapsed += delta;

		// Deal with teh current frame
		if (elapsed > timeStep)
		{
			// Animate 1 frame
			if (!pause)	update(elapsed);
			else		if (method != 7)	update(0);		// To compare interpolation methods

			elapsed = 0;	// Reset the elapsed time
		}

		render(window);				// Draw one frame
		glfwSwapBuffers(window); 	// Swap buffers
		glfwPollEvents();			// Events
	}

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

void
init(const char* filename)
{
	// Read a mesh
	cout << "Reading " << filename << endl;
	readMesh(filename, vertex, normal, face);

	// Initial transformation of the mesh
	T.setIdentity();

	// Position
	p1 = Vector3f(-1, 0.5, 2);
	p2 = Vector3f(2, 0.5, -1);

	// Position differences
	Vector3f	v = p2 - p1;
	cout << "Distance = " << v.norm() << endl << endl;

	// Orientations: no rotation and 181 degree rotation
	Vector3f	axis = Vector3f(1, 1, 1).normalized();
	float		theta = 181;
	q1 = Quaternionf(AngleAxisf(0, axis));
	cout << "q1 = " << q1.w() << ", " << q1.vec().transpose() << endl << endl;

	q2 = Quaternionf(AngleAxisf(theta * float(M_PI) / 180, axis));
	cout << "q2 from theta = " << theta << " and axis = " << axis.transpose() << endl;
	cout << "q2 = " << q2.w() << ", " << q2.vec().transpose() << endl;

	// Orientation difference. AngleAxis deals with antipodal equivalence.
	AngleAxisf	aa(q1.inverse() * q2);	// ------- (1)
	cout << "q1 -> q2: Angle = " << aa.angle() / M_PI * 180 << " degree, ";
	cout << "Axis = " << aa.axis().transpose() << endl << endl;

	// q3 (= -q2) represent the same orientation with q2 owing to antipodal equivalence.
	Quaternionf	q3(-q2.w(), -q2.x(), -q2.y(), -q2.z());
	cout << "q3 from -q2" << endl;
	cout << "q3 = " << q3.w() << ", " << q3.vec().transpose() << endl;

	// AngleAxis already deals with antipodal equivalence.
	// i.e. q1.inverse() * (-q2) was already employed in (1).
	// It can be verified by the following computation.
	aa = AngleAxisf(q1.inverse() * q3);
	cout << "q1 -> q3: Angle = " << aa.angle() / M_PI * 180 << " degree, ";
	cout << "Axis = " << aa.axis().transpose() << endl << endl;
}

Matrix3f
rlerp(float t, Quaternionf& q1, Quaternionf& q2)
{
	// Linear interpolation of the rotation parts in the 4x4 homogenous matrix
	/*if (!pause)
	{
		cout << "q1" << endl;
		cout << (1 - t) * Matrix3f(q1) << endl;
		cout << "q2" << endl;
		cout << t * Matrix3f(q2) << endl;
		cout << "q" << endl;
		cout << (1 - t) * Matrix3f(q1) + t * Matrix3f(q2) << endl;
		cout << endl;
	}*/


	Matrix3f	R = (1 - t) * Matrix3f(q1) + t * Matrix3f(q2);

	return	R;
}

Matrix3f
qlerp(float t, Quaternionf& q1, Quaternionf& q2)
{
	Quaternionf	q;

	// Linear interpolation of quaternions
	q.w() = (1 - t) * q1.w() + t * q2.w();
	q.x() = (1 - t) * q1.x() + t * q2.x();
	q.y() = (1 - t) * q1.y() + t * q2.y();
	q.z() = (1 - t) * q1.z() + t * q2.z();

	// Normalization
	q.normalize();

	/*if (!pause)
	{
		cout << "q1" << endl;
		cout << "q1 = " << q1.w() << ", " << q1.vec().transpose() << endl;
		cout << "q2" << endl;
		cout << "q2 = " << q2.w() << ", " << q2.vec().transpose() << endl;
		cout << "q" << endl;
		cout << "q = " << q.w() << ", " << q.vec().transpose() << endl;
		cout << endl;
	}*/


	return Matrix3f(q);
}

void
update(float elapsed)
{
	// Time update
	currTime += elapsed;

	// Time t oscillating in [0, 1] with interval
	int		n = int(currTime / interval);				// n-th iteration
	float	s = (currTime - n * interval) / interval;	// s increasing in [0, 1]
	float	t = (s < 0.5) ? 2 * s : 2 * (1 - s);		// t oscillating in [0, 1]
	// cout << currTime << "\t" << n << "\t" << s << "\t" << t << endl;

	// Linear interpolation of the given two positions in all cases
	// Set the translational part in the 4x4 homogenous matrix
	Vector3f	p = (1 - t) * p1 + t * p2;
	T.block<3, 1>(0, 3) = p;

	// Linear interpolation of the given two rotation matrices
	// Set the rotational part in the 4x4 homogenous matrix
	if (method == 2)	T.block<3, 3>(0, 0) = rlerp(t, q1, q2);

	// Linear interpolation of the given two unit quaternions
	if (method == 3)	T.block<3, 3>(0, 0) = qlerp(t, q1, q2);

	// Sphererical linear interpolation of the given two orientations
	if (method >= 4)
	{
		Quaternionf q;

		// Slerp provided by Eigen
		if (method == 4)	q = q1.slerp(t, q2);

		// Slerp implementation
		if (method == 5 || method == 6)
		{
			// Eigen does not provide the logarithmic map.
			// Instead, AngleAxis can be exploited.
			AngleAxisf	aa(q1.conjugate() * q2);
			aa.angle() *= t;

			if (method == 5)	q = q1 * aa;				// Quaternion AngleAxis Mult
			if (method == 6)	q = q1 * Quaternionf(aa);	// Quaternion Multiplization
		}

		// Set the 3x3 rotational part of the 4x4 homogenous matrix
		T.block<3, 3>(0, 0) = Matrix3f(q);
	}
}

// Draw a sphere after setting up its material
void
drawMesh(int flag)
{
	// Material
	GLfloat mat_ambient[4] = { 0.10f, 0.10f, 0.10f, 1 };
	GLfloat mat_diffuse[4] = { 0.95f, 0.95f, 0.95f, 1 };
	GLfloat mat_specular[4] = { 0.10f, 0.10f, 0.10f, 1 };
	GLfloat mat_shininess = 128;

	if (method == 7)
	{
		if (flag == 0) { glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); mat_diffuse[1] = 0.1f; mat_diffuse[2] = 0.1f; }
		else { glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); mat_diffuse[0] = 0.1f; mat_diffuse[1] = 0.1f; }
	}

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);



	// Mesh
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < face.cols(); i++)
	{
		glNormal3fv(normal.col(face(0, i)).data());
		glVertex3fv(vertex.col(face(0, i)).data());

		glNormal3fv(normal.col(face(1, i)).data());
		glVertex3fv(vertex.col(face(1, i)).data());

		glNormal3fv(normal.col(face(2, i)).data());
		glVertex3fv(vertex.col(face(2, i)).data());
	}
	glEnd();
}

void
updateComparison(float Sequence, int flag)
{
	// Linear interpolation of the given two positions in all cases
	// Set the translational part in the 4x4 homogenous matrix
	Vector3f	p = (1 - Sequence) * p1 + Sequence * p2;
	T.block<3, 1>(0, 3) = p;

	// Linear interpolation of the given two rotation matrices
	// Set the rotational part in the 4x4 homogenous matrix
	
	if (flag == 0)	T.block<3, 3>(0, 0) = rlerp(Sequence, q1, q2);

	if (flag == 1)
	{
		Quaternionf q;
		q = q1.slerp(Sequence, q2);
		T.block<3, 3>(0, 0) = Matrix3f(q);
	}
}

void
render(GLFWwindow* window)
{
	// Backgroud color
	glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2], up[0], up[1], up[2]);

	// Axes
	glDisable(GL_LIGHTING);
	drawAxes(AXIS_LENGTH, AXIS_LINE_WIDTH);

	// Lighting
	setupLight();

	// Apply rotation and translation. Multiply T to the current matrix of OpenGL.
	// OpenGL employs right multiplication of matrix. T need to be transposed and then
	// fed in the row-major order. Meanwhile, Eigen represents T in teh column-major order.
	// Thus, T can be fed into OpenGL without any modifications
	// Draw the mesh after setting up the material


	if (method == 7)
	{
		for (int i = 0; i < 11; i++)
		{
			glPushMatrix();
			updateComparison(float(i) / 10, 0);
			glMultMatrixf(T.data());
			drawMesh(0);
			glPopMatrix();
		}

		for (int i = 0; i < 11; i++)
		{
			glPushMatrix();
			updateComparison(float(i) / 10, 1);
			glMultMatrixf(T.data());
			drawMesh(1);
			glPopMatrix();
		}
	}

	else
	{
		glMultMatrixf(T.data());
		drawMesh(1);
	}
}

// Light
void
setupLight()
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	GLfloat ambient[4] = { 0.1f, 0.1f, 0.1f, 1 };
	GLfloat diffuse[4] = { 1.0f, 1.0f, 1.0f, 1 };
	GLfloat specular[4] = { 1.0f, 1.0f, 1.0f, 1 };

	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light.data());
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

		// Numeric
		case GLFW_KEY_1:	method = 1; cout << methodString[method - 1] << endl;	break;
		case GLFW_KEY_2:	method = 2; cout << methodString[method - 1] << endl;	break;
		case GLFW_KEY_3:	method = 3; cout << methodString[method - 1] << endl;	break;
		case GLFW_KEY_4:	method = 4; cout << methodString[method - 1] << endl;	break;
		case GLFW_KEY_5:	method = 5; cout << methodString[method - 1] << endl;	break;
		case GLFW_KEY_6:	method = 6; cout << methodString[method - 1] << endl;	break;
		case GLFW_KEY_7:	method = 7; cout << "Visual comparison for exercise" << endl;	break;

			// Play/pause toggle
		case GLFW_KEY_SPACE:	pause = !pause; break;
		}
	}
}
