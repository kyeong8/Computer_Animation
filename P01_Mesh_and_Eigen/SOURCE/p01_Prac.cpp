#include "glSetup.h"
#include "Prac_mesh.h"

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
void	setupColoredMaterial(const Vector3f& color);
void	render(GLFWwindow* window);
void	reshape(GLFWwindow* window, int w, int h);
void	keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);

// Camera configuration for ray tracing
Vector3f			eye(2, 2, 2);
Matrix<float,3,1>	center(0, 0.2f, 0);	// Vector3f
Vector3f			up(0, 1, 0);

// Light configuration
Vector4f	light(5.0, 5.0, 0.0, 1);	// Light position

// Play configuraion
bool	pause = true;
float	timeStep = 1.0f / 120;		// 120fps
float	period = 8.0;
float	theta = 0;

// Eigen/OpenGL rotation
bool	eigen = false;

// Global coordinate frame
bool	axes = false;
float	AXIS_LENGTH = 1.5;
float	AXIS_LINE_WIDTH = 2;

// Colors
GLfloat bgColor[4] = { 1, 1, 1, 1 };

// Default mesh file name
const char* defaultMeshFileName = "m01_bunny.off";

// Display style
bool	aaEnabled = true;	// Antialiasing
bool	bfcEnabled = true;	// Back face culling

// Mesh
MatrixXf							vertex;			// Position
Matrix <float, Dynamic, Dynamic>	faceNormal;		// Face normal vector
MatrixXf							vertexNormal;	// Vertex normal vector
ArrayXXi							face;			// Index

// Mesh with shrunken faces
MatrixXf	faceVertex;

// Control variables
bool	faceWithGapMesh = true;
bool	useFaceNormal = true;
float	gap = 0.1f;

int
main(int argc, char* argv[])
{
	// Mesh filename
	const char* filename;
	if (argc >= 2)	filename = argv[1];
	else            filename = defaultMeshFileName;
	
	// Initialize the OpenGL system
	GLFWwindow* window = initializeOpenGL(argc, argv, bgColor);
	if (window == NULL) return	-1;

	// Callbacks
	glfwSetKeyCallback(window, keyboard);

	// Depth test
	glEnable(GL_DEPTH_TEST);

	// Normal vectors are normalized after transformation.
	glEnable(GL_NORMALIZE);

	// Viewport and perspective setting
	reshape(window, windowW, windowH);

	// Initialization - Main loop - Finalization
	init(filename);

	// Main loop
	float	previous = (float)glfwGetTime();
	float	elapsed = 0;
	while (!glfwWindowShouldClose(window))
	{
		glfwPollEvents();			// Events

		// Time passed during a single loop
		float	now = (float)glfwGetTime();
		float	delta = now - previous;
		previous = now;

		// Time passed after the previous frame
		elapsed += delta;

		// Deal with the current frame
		if (elapsed > timeStep)
		{
			if (!pause) theta += float(2.0 * M_PI) / period * elapsed;

			elapsed = 0;	// Rest the elapsed time
		}

		render(window);				// Draw one frame
		glfwSwapBuffers(window);	// Swap buffers
	}

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

void
buildShrunkenFaces(const MatrixXf& vertex, MatrixXf& faceVertex)
{
	// Face mesh with # faces and (3 x # faces) vertices
	faceVertex.resize(3, 3 * face.cols());
	for (int i = 0; i < face.cols(); i++)
	{
		// Center of the i-th face
		Vector3f	center(0, 0, 0);
		for (int j = 0; j < 3; j++)
			center += vertex.col(face(j, i));
		center /= 3.0f;

		// Build 3 shrunken vertices per triangle
		for (int j = 0; j < 3; j++)
		{
			const Vector3f& p = vertex.col(face(j, i));
			faceVertex.col(3 * i + j) = p + gap * (center - p);
		}
	}
}

void
init(const char* filename)
{
	// Read a mesh
	cout << "Reading " << filename << endl;
	readMesh(filename, vertex, face, faceNormal, vertexNormal);

	// Shrunken face mesh
	buildShrunkenFaces(vertex, faceVertex);

	// Usage
	cout << endl;
	cout << "Keyboard Input: space for play/pause" << endl;
	cout << "Keyboard Input: x for axes on/off" << endl;
	cout << "Keyboard Input: q/esc for quit" << endl;
	cout << endl;
	cout << "Keyboard Input: g for face with gap mesh/mesh" << endl;
	cout << "Keyboard Input: up/down to increase/decrease the gap between faces" << endl;
	cout << "Keyboard Input: n for vertex/face normal" << endl;
	cout << "Keyboard Input: e for rotation with Eigen" << endl;
	cout << "Keyboard Input: a for antialiasing on/off" << endl;
	cout << "Keyboard Input: b for back face culling on/off" << endl;
}

// Draw a mesh after setting up its material
void
drawMesh()
{
	// Material
	setupColoredMaterial(Vector3f(0.95f, 0.95f, 0.95f));

	// Mesh
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < face.cols(); i++)
	{
		glNormal3fv(vertexNormal.col(face(0, i)).data());
		glVertex3fv(vertex.col(face(0, i)).data());

		glNormal3fv(vertexNormal.col(face(1, i)).data());
		glVertex3fv(vertex.col(face(1, i)).data());

		glNormal3fv(vertexNormal.col(face(2, i)).data());
		glVertex3fv(vertex.col(face(2, i)).data());
	}
	glEnd();
}

// Draw shrunken faces
void
drawShrunkenFaces()
{
	// Material
	setupColoredMaterial(Vector3f(0.95f, 0.95f, 0.95f));

	// Mesh
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < face.cols(); i++)
	{
		if (useFaceNormal)	glNormal3fv(faceNormal.col(i).data());
		for (int j = 0; j < 3; j++)
		{
			if (!useFaceNormal) glNormal3fv(vertexNormal.col(face(j, i)).data());
			glVertex3fv(faceVertex.col(3 * i + j).data());
		}
	}
	glEnd();
}

void
render(GLFWwindow* window)
{
	// Antialiasing
	if (aaEnabled)	glEnable(GL_MULTISAMPLE);
	else            glDisable(GL_MULTISAMPLE);

	// Back face culling
	if (bfcEnabled)
	{
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glFrontFace(GL_CCW);
	}
	else    glDisable(GL_CULL_FACE);

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
		drawAxes(AXIS_LENGTH, AXIS_LINE_WIDTH);
	}

	// Lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	setupLight();

	// Rotation about the y-axis
	if (eigen)
	{
		// 4x4 homogeneous identity matrix
		Matrix4f	T;	T.setIdentity();

		// Rotation matrix: theta in radian
		T.block<3, 3>(0, 0) = Matrix3f(AngleAxisf(theta, Vector3f::UnitY()));

		// Translation vector
		T.block<3, 1>(0, 3) = Vector3f(0.0f, 0.0f, 0.0f);

		// Apply rotation and translation. Multiply T to the current matrix of OpenGL.
		// OpenGL employs right multiplication of matrix. T need to be transposed and then
		// fed in the row-major order. Meanwhile, Eigen represents T in the column-major order.
		// Thus, T can be fed into OpenGL without any modifications/
		glMultMatrixf(T.data());
	}
	else {
		glRotatef(theta / float(M_PI) * 180.0f, 0, 1, 0);	// Radian to degree
	}

	// Draw the mesh after setting up the material
	if (faceWithGapMesh)	drawShrunkenFaces();
	else                    drawMesh();
}

// Material
void
setupColoredMaterial(const Vector3f& color)
{
	// Material
	GLfloat mat_ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat mat_diffuse[4] = { color[0], color[1], color[2], 1.0f };
	GLfloat mat_specular[4] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat mat_shininess = 100.0f;

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
}

// Light
void
setupLight()
{
	GLfloat ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat diffuse[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat specular[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

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

		// Play on/off
		case GLFW_KEY_SPACE:	pause = !pause; break;

		// Eigen/OpenGL rotation
		case GLFW_KEY_E:		eigen = !eigen; break;
			
		// Face with gap mesh/mesh
		case GLFW_KEY_G:		faceWithGapMesh = !faceWithGapMesh; break;

		// Gap increase/decrease
		case GLFW_KEY_UP:
			gap = min(gap + 0.05f, 0.5f);
			buildShrunkenFaces(vertex, faceVertex);
			break;
		case GLFW_KEY_DOWN:
			gap = min(gap - 0.05f, 0.5f);
			buildShrunkenFaces(vertex, faceVertex);
			break;

		// Normal vector
		case GLFW_KEY_N:	useFaceNormal = !useFaceNormal; break;

		// Antialiasing on.off
		case GLFW_KEY_A:	aaEnabled = !aaEnabled; break;

		// Back face culling
		case GLFW_KEY_B:	bfcEnabled = !bfcEnabled; break;

		// Axes on/off
		case GLFW_KEY_X:	axes = !axes;	break;
		}
	}
}
