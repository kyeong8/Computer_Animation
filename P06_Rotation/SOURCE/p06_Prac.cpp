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

void	update();
void	render(GLFWwindow* window);
void	reshape(GLFWwindow* window, int w, int h);
void	keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);

// Play configuration
bool	pause = true;

// Camera configuration
Vector3f	eye(0, 0, 3);
Vector3f	center(0, 0, 0);
Vector3f	up(0, 1, 0);

// Light configuration
Vector4f	light(5.0, 5.0, 5.0, 0);		// Light direction

// Global coordinate frame
bool	axes = false;
float	AXIS_LENGTH = 1.5f;
float	AXIS_LINE_WIDTH = 2;

// Colors
GLfloat bgColor[4] = { 1, 1, 1, 1 };

// Default mesh file name
const char* defaultMeshFileName = "m01_bunny.off";

// Basic rotation/incremental rotation
bool	incremental = false;

int
main(int argc, char* argv[])
{
	// Immediate mode to verify the artifacts ASAP
	vsync = 0;

	// Filename for deformable body configuration
	const char* filename;
	if (argc >= 2)	filename = argv[1];
	else            filename = defaultMeshFileName;

	// Orthographic viewing
	perspectiveView = false;

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

	// Initialization - Main loop - Finalization
	init(filename);

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		if (!pause)	update();

		render(window);		// Draw one frame
		glfwSwapBuffers(window); 	// Swap buffers
		glfwPollEvents();			// Events
	}

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

// Mesh
MatrixXf	vertexR, vertexQ, vertexO;
MatrixXf	normalR, normalQ, normalO;
ArrayXXi	face;

// Time
int	frame = 0;

// Orientation
int	Nacc = 1;

Matrix4f	Tc[3];	// Rotation without/with normalization, quaternion with normalization
Matrix4f	Ti[3];	// Initial transformation
Matrix3f	R, Rn;	// Rotation matrix without/with normalization
Quaternionf	q;		// Quaternion with normalization
Vector3f	axis;

void
init(const char* filename)
{
	// Read a mesh
	cout << "Reading " << filename << endl;
	readMesh(filename, vertexO, normalO, face);

	// To rotate each vertex and normal vectors in basic rotation
	vertexR = vertexQ = vertexO;
	normalR = normalQ = normalO;

	// Initial orientation of the mesh
	Affine3f	T;
	T = Translation3f(-1.2f, 0, 0) * Scaling(0.4f, 0.4f, 0.4f);	Tc[0] = Ti[0] = T.matrix();
	T = Translation3f( 0.0f, 0, 0) * Scaling(0.4f, 0.4f, 0.4f);	Tc[1] = Ti[1] = T.matrix();
	T = Translation3f( 1.2f, 0, 0) * Scaling(0.4f, 0.4f, 0.4f);	Tc[2] = Ti[2] = T.matrix();

	R.setIdentity();
	Rn.setIdentity();
	q.setIdentity();

	// Usage
	cout << endl;
	cout << "Keyboard Input: space for play/pause" << endl;
	cout << "Keyboard Input: q/esc for quit" << endl;
	cout << endl;
	cout << "Keyboard Input: i for basic/incremental rotation" << endl;
	cout << "Keyboard Input: a for acceleration in incremental rotation" << endl;
	cout << "Keyboard Input: x for axes on/off" << endl;
}

void
basicRotation()
{
	float	angleInc = float(M_PI) / 1200;	// In radian

	// Generate a random axis at every 2,400 frames
	if (frame % 2400 == 0)	axis = Vector3f::Random().normalized();

	// Rotation matrix from angle-axis rotation - (2)
	{
		AngleAxisf	aa(angleInc * frame, axis);
		Matrix3f	R(aa);

		for (int i = 0; i < vertexO.cols(); i++)
		{
			vertexR.col(i) = R * vertexO.col(i);	// Vertex position
			normalR.col(i) = R * normalO.col(i);	// Normal vector
		}
	}

	// Quaternion - (3)
	{
		Quaternionf q;
		q.w() = cos(angleInc * frame / 2.0f);
		q.vec() = axis * sin(angleInc * frame / 2.0);

		Quaternionf P;		P.w() = 0;
		Quaternionf P_new;
		for (int i = 0; i < vertexO.cols(); i++)
		{
			// Vertex position
			P.vec() = vertexO.col(i);
			P_new = q * P * q.conjugate();	// q.conjugate() = q.inverse()
			vertexQ.col(i) = P_new.vec();

			// Normal vector
			P.vec() = normalO.col(i);
			P_new = q * P * q.inverse();
			normalQ.col(i) = P_new.vec();
		}
	}

	// Transformation - (1)
	{
		AngleAxisf	aa(angleInc * frame, axis);
		Matrix3f	R(aa);

		Matrix4f	Tt;	Tt.setIdentity(); Tt.block<3, 3>(0, 0) = R;
		Tc[2] = Ti[2] * Tt;

		// Conversion examples
		if (0)
		{
			Quaternionf	q(aa);
			Matrix3f	R_from_q(q);
			Quaternionf	q_from_R(R);

			AngleAxisf	aa_from_q(q);
			AngleAxisf	aa_from_R(R);
		}
	}

	frame++;
}

// Incremental rotation
void
incrementalRotation()
{
	float	angleInc = float(M_PI) / 1200;	// In radian

	for (int i = 0; i < Nacc; i++)
	{
		// Generate a random axis a every 240 frames
		if (frame % 2400 == 0)	axis = Vector3f::Random().normalized();

		// Angle-Axis rotation
		AngleAxisf	aa(angleInc, axis);

		// Rotation matrix without normalization
		{
			R = Matrix3f(aa) * R;

			Matrix4f	Tt;	Tt.setIdentity(); Tt.block<3, 3>(0, 0) = R;
			Tc[0] = Ti[0] * Tt;
		}

		// Rotation matrix with normalization
		{
			Rn = Matrix3f(aa) * Rn;

			// Normalize the rotation matrix
			Rn /= pow(fabs(Rn.determinant()), 1.0f / 3.0f);
			
			Matrix4f	Tt; Tt.setIdentity(); Tt.block<3, 3>(0, 0) = Rn;
			Tc[1] = Ti[1] * Tt;
		}

		// Quaternion with normalization
		{
			q = Quaternionf(aa) * q;

			// Normalize the quaternion
			q.normalize();

			// Convert the quaternion into the rotation matrix
			Matrix4f	Tt; Tt.setIdentity(); Tt.block<3, 3>(0, 0) = Matrix3f(q);
			Tc[2] = Ti[2] * Tt;
		}

		// Check the integrity at every 2,400 frames
		if (frame % 2400 == 0)
		{
			Vector3f	x = R.col(0), y = R.col(1), z = R.col(2);

			cout << endl;
			cout << "Unitary:\t" << x.norm() << ", " << y.norm() << ", " << z.norm() << endl;
			cout << "Orthogonal:\t" << x.dot(y) << ", " << y.dot(z) << ", " << z.dot(x) << endl;
			cout << "Determinant:\t" << R.determinant() << endl;
		}

		frame++;
	}
}

void
update()
{
	if (incremental)	incrementalRotation();
	else                basicRotation();
}

// Draw a mesh after setting up its material
void
drawMesh(const MatrixXf& vertex, const MatrixXf& normal)
{
	// Material
	GLfloat mat_ambient[4] = { 0.1f, 0.1f, 0.1f, 1 };
	GLfloat mat_diffuse[4] = { 0.95f, 0.95f, 0.95f, 1 };
	GLfloat mat_specular[4] = { 0.5f, 0.5f, 0.5f, 1 };
	GLfloat mat_shininess = 128;

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
	if (axes)
	{
		glDisable(GL_LIGHTING);
		drawAxes(AXIS_LENGTH, AXIS_LINE_WIDTH * dpiScaling);
	}

	// Lighting
	setupLight();

	// Draw the mesh after setting up the material
	if (incremental)
	{
		for (int i = 0; i < 3; i++)
		{
			glPushMatrix();

			// Apply the rotation;
			// Eigen and OpenGL employ the same colum-major representation.
			glMultMatrixf(Tc[i].data());

			drawMesh(vertexO, normalO);
			glPopMatrix();
		}
	}
	else {
		glPushMatrix();
		glMultMatrixf(Ti[0].data());
		drawMesh(vertexR, normalR);
		glPopMatrix();

		glPushMatrix();
		glMultMatrixf(Ti[1].data());
		drawMesh(vertexQ, normalQ);
		glPopMatrix();

		glPushMatrix();
		glMultMatrixf(Tc[2].data());
		drawMesh(vertexO, normalO);
		glPopMatrix();
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

			// Basic/incremental rotation
		case GLFW_KEY_I:		incremental = !incremental; break;

			// Acceleration
		case GLFW_KEY_A:		Nacc = (Nacc == 1) ? 100000 : 1;	break;

			// Axes on/off
		case GLFW_KEY_X:		axes = !axes;	break;

			// Play/pause toggle
		case GLFW_KEY_SPACE:	pause = !pause; break;
		}
	}
}
