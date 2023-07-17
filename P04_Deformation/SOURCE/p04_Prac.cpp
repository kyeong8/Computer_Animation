
#include "glSetup.h"
#include "glShader.h"
#include "mesh.h"

#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>
using namespace std;

#ifdef _WIN32
#define _USE_MATH_DEFINES	// To include the definition of M_PI in math.h
#endif

#include <math.h>

void	update();
void	render(GLFWwindow* window);
void	keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);

// Camera configuration
Vector3f	eye(0, 0, 1.75);
Vector3f	center(0, 0, 0);
Vector3f	up(0, 1, 0);

// Light configuration
Vector3f	light1(0.0, 0.0, 5.0);
Vector3f	light2(0.0, 5.0, 5.0);

// Colors
GLfloat	bgColor[4] = { 1, 1, 1, 1 };

// Display style
bool	aaEnabled = true;

// Model, view, projection matrix
Matrix4f	ViewMatrix, ProjectionMatrix;

// Play configuration
bool	pause = true;

// Control parameters
float	tau = 0;			// twisting in twist deformer and phase in the wave deformer
bool	CCW = true;			// CCW or CW rotation
float	frequency = 40.0;	// Spatial frequence in the wave deformer

// Program and shaders
struct Program
{
	GLuint	vs;		// Vertex shader
	GLuint	fs;		// Fragment shader
	GLuint	pg;		// Program

	Program() { vs = 0; fs = 0; pg = 0; }

	void
		create(const char* vertexShaderFileName, const char* fragmentShaderFileName)
	{
		createShaders(vertexShaderFileName, fragmentShaderFileName, pg, vs, fs);
	}

	void destroy() { deleteShaders(pg, vs, fs); }
};

Program pgTwist;		// Program for the twist deformer
Program pgWave;			// Program for the wave deformer

// Geometry
struct Geometry
{
	GLuint vao;				// Vertex array object
	GLuint indexId;			// Buffer for triangle inddices
	GLuint vertexId;		// Buffer for vertex positions
	GLuint normalId;		// Buffer for normal vectors

	int	   numTris;			// # of triangles

	Geometry()
	{
		vao = 0;
		indexId = 0;
		vertexId = 0;
		normalId = 0;
	}
};

Geometry	plane[4];	// VAO and VBO for nxn planar meshes

const char* planeFileName[4] = {
	"m01_plane32.off",	// level 0
	"m01_plane64.off",	// level 1
	"m01_plane128.off",	// level 2
	"m01_plane256.off",	// level 3
};

int level = 3;

// Wireframe view
bool	wireframe = false;

// Example 0 for the twist deformer and 1 for the wave deformer
int		example = 1;

void
reshapeModernOpenGL(GLFWwindow* window, int w, int h)
{
	// Window configuration
	aspect = (float)w / h;
	windowW = w;
	windowH = h;

	// Viewport
	glViewport(0, 0, w, h);

	// Projection matrix
	float	fovyR = fovy * float(M_PI) / 180.0f;	// fovy in radian
	float	nearDist = 0.01;
	float	farDist = 10.0;
	ProjectionMatrix = perspective<float>(fovyR, aspect, nearDist, farDist);

	// The Screen size is required for mouse interaction.
	glfwGetWindowSize(window, &screenW, &screenH);
	cerr << "reshape(" << w << ", " << h << ")";
	cerr << " with screen " << screenW << " x " << screenH << endl;
}

int
main(int argc, char* argv[])
{
	// Initialize the OpenGL system: true for modern OpenGL
	GLFWwindow* window = initializeOpenGL(argc, argv, bgColor, true);
	if (window == NULL) return	-1;

	// Viewport and perspective setting
	reshapeModernOpenGL(window, windowW, windowH);

	// Callbacks
	glfwSetFramebufferSizeCallback(window, reshapeModernOpenGL);
	glfwSetKeyCallback(window, keyboard);

	// Depth test
	glEnable(GL_DEPTH_TEST);

	// Initialization
	{
		// Create shaders, VAO and VBO for Gouraud and Phong
		pgTwist.create("sv04_twist.glsl", "sf02_Phong.glsl");
		pgWave.create("sv04_wave.glsl", "sf02_Phong.glsl");

		ArrayXXi	face;
		MatrixXf	vertex;
		MatrixXf	normal;

		for (int i = 0; i < 4; i++)
		{
			// Create VAO and VBO for a single quad
			createVBO(plane[i].vao, plane[i].indexId, plane[i].vertexId, plane[i].normalId);

			// Load the mesh
			readMesh(planeFileName[i], vertex, normal, face);

			// Upload the data into the buffers
			plane[i].numTris = uploadMesh2VBO(face, vertex, normal,
					plane[i].vao, plane[i].indexId, plane[i].vertexId, plane[i].normalId);
		}
	}

	// Usage
	cout << endl;
	cout << "Keyboard Input: space for play/pause" << endl;
	cout << "Keyboard Input: q/esc for quit" << endl;
	cout << endl;
	cout << "Keyboard Input: t to toggle twist/wave deformers" << endl;
	cout << "Keyboard Input: w to toggle the wireframe view" << endl;
	cout << "Keyboard Input: up/down to increase/decrease the spatial frequency" << endl;
	cout << "Keyboard Input: r for reversting the direction" << endl;
	cout << "Keyboard Input: i for initialization" << endl;
	cout << endl;
	cout << "Keyboard Input: 1 for the 32 x 32 planar mesh" << endl;
	cout << "Keyboard Input: 2 for the 64 x 64 planar mesh" << endl;
	cout << "Keyboard Input: 3 for the 128 x 128 planar mesh" << endl;
	cout << "Keyboard Input: 4 for the 256 x 256 planar mesh" << endl;

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		// Update one frame if not paused
		if (!pause) update();

		// Draw one frame
		render(window);

		glfwSwapBuffers(window);	// Swap buffers
		glfwPollEvents();			// Events
	}

	// Finalization
	{
		// Delete VBO and shaders
		for (int i = 0; i < 4; i++)
			deleteVBO(plane[i].vao, plane[i].indexId, plane[i].vertexId, plane[i].normalId);

		pgTwist.destroy();
		pgWave.destroy();
	}

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

void
update()
{
	if (CCW)	tau += 1.0f / 60.0f;
	else        tau -= 1.0f / 60.0f;
}

void
setUniformMVP(GLuint program, Matrix4f& M, Matrix4f& V, Matrix4f& P)
{
	// ModelView matrix
	Matrix4f	ModelViewMatrix = V * M;
	setUniform(program, "ModelViewMatrix", ModelViewMatrix);

	// Normal matrix
	Matrix3f	NormalMatrix = ModelViewMatrix.block<3, 3>(0, 0).inverse().transpose();
	setUniform(program, "NormalMatrix", NormalMatrix);

	// ModelViewProjection matrix in the vertex shader
	Matrix4f	ModelViewProjectionMatrix = P * ModelViewMatrix;
	setUniform(program, "ModelViewProjectionMatrix", ModelViewProjectionMatrix);
}

void
render(GLFWwindow* window)
{
	// Antialiasing
	if (aaEnabled)	glEnable(GL_MULTISAMPLE);
	else            glDisable(GL_MULTISAMPLE);

	// Polygon/line
	if (wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else           glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// Backgroud color
	glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Camera configuration
	ViewMatrix = lookAt<float>(eye, center, up);

	if (example == 0)
	{
		// Model  matrix
		Matrix4f	ModelMatrix;
		ModelMatrix.setIdentity();

		// Model, view, projection matrices
		setUniformMVP(pgTwist.pg, ModelMatrix, ViewMatrix, ProjectionMatrix);

		// Light position in the fragment shader, represented in the view coordinate system
		Vector3f	l = ViewMatrix.block<3, 3>(0, 0) * light1 + ViewMatrix.block<3, 1>(0, 3);
		setUniform(pgTwist.pg, "LightPosition", l);

		// Draw objects: only the bunny in this case.
		{
			// Material in dependent of the object
			setUniform(pgTwist.pg, "Ka", Vector3f(0.10f, 0.10f, 0.10f));
			setUniform(pgTwist.pg, "Kd", Vector3f(0.75f, 0.75f, 0.75f));
			setUniform(pgTwist.pg, "Ks", Vector3f(0.00f, 0.00f, 0.00f));
			setUniform(pgTwist.pg, "Shininess", 128.0f);

			// Twisting
			setUniform(pgTwist.pg, "twisting", tau);

			// Draw the mesh using the program and the vertex buffer object
			glUseProgram(pgTwist.pg);
			drawVBO(plane[level].vao, plane[level].numTris);
		}
	}
	else if (example == 1)
	{
		// Model  matrix
		Affine3f	T;	T =  Matrix3f(AngleAxisf(-float(M_PI)/3.0f, Vector3f::UnitX()))
						  * Scaling(1.5f, 1.5f, 1.5f);
		Matrix4f	ModelMatrix = T.matrix();

		// Model, view, projection matrices
		setUniformMVP(pgWave.pg, ModelMatrix, ViewMatrix, ProjectionMatrix);

		// Light position in the fragment shader, represented in the view coordinate system
		Vector3f	l = ViewMatrix.block<3, 3>(0, 0) * light2 + ViewMatrix.block<3, 1>(0, 3);
		setUniform(pgWave.pg, "LightPosition", l);

		// Draw objects: only the bunny in this case.
		{
			// Material in dependent of the object
			setUniform(pgWave.pg, "Ka", Vector3f(0.10f, 0.10f, 0.10f));
			setUniform(pgWave.pg, "Kd", Vector3f(0.75f, 0.75f, 0.75f));
			setUniform(pgWave.pg, "Ks", Vector3f(0.10f, 0.10f, 0.10f));
			setUniform(pgWave.pg, "Shininess", 128.0f);

			// Phase
			setUniform(pgWave.pg, "phase", 4 * tau);

			// Spatial frequency
			setUniform(pgWave.pg, "F", frequency);

			// Draw the mesh using the program and the vertex buffer object
			glUseProgram(pgWave.pg);
			drawVBO(plane[level].vao, plane[level].numTris);
		}
	}

	// Check the status
	isOK("render()", __FILE__, __LINE__);
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
		case GLFW_KEY_SPACE:	pause = !pause;		break;		// Play on/off
		case GLFW_KEY_I:		tau = 0;			break;		// Initialization
		case GLFW_KEY_R:		CCW = !CCW;			break;		// Direction

			// Examples
		case GLFW_KEY_T:	example = (example + 1) % 2;	break;

			// Level
		case GLFW_KEY_1:	level = 0; break;
		case GLFW_KEY_2:	level = 1; break;
		case GLFW_KEY_3:	level = 2; break;
		case GLFW_KEY_4:	level = 3; break;

			// Spatial frequency in the wave deformer
		case GLFW_KEY_UP:		frequency += 1.0f;		break;
		case GLFW_KEY_DOWN:		frequency -= 1.0f;		break;

			// Drawing in wireframe on/off
		case GLFW_KEY_W:	wireframe = !wireframe;	break;
		}
	}
}
