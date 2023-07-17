
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

void	update(Matrix4f& ModelMatrix);
void	render(GLFWwindow* window, GLuint program,
				GLuint vao, int numTris, Matrix4f& ModelMatrix);
void	keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);

// Camera configuration
Vector3f	eye(2, 2, 2);
Vector3f	center(0, 0.2f, 0);
Vector3f	up(0, 1, 0);

// Light configuration
Vector3f	light0(5.0, 5.0, -1.0);
Vector3f	light1(1.0, 5.0, 5.0);

// Colors
GLfloat	bgColor[4] = { 1, 1, 1, 1 };

// Default mesh file name
const char* defaultMeshFileName = "m01_bunny.off";

// Display style
bool	aaEnabled = true;	// Antialiasing

// Model, view, projection matrix
Matrix4f	ViewMatrix, ProjectionMatrix;

// Play configuration
bool	pause = true;
int		frame = 0;

// Command mode
char	command = 'n';	// Shininess

// Material properties
Vector3f	Ka(0.10f, 0.10f, 0.10f);
Vector3f	Kd(0.75f, 0.75f, 0.75f);
Vector3f	Ks(0.50f, 0.50f, 0.50f);
float		Shininess = 128.0f;

// Gouraud/Phong
bool		phong = true;

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
	float	nearDist = 1.0;
	float	farDist = 20.0;
	ProjectionMatrix = perspective<float>(fovyR, aspect, nearDist, farDist);

	// The Screen size is required for mouse interaction.
	glfwGetWindowSize(window, &screenW, &screenH);
	cerr << "reshape(" << w << ", " << h << ")";
	cerr << " with screen " << screenW << " x " << screenH << endl;
}

int
main(int argc, char* argv[])
{
	// Mesh filename
	const char* filename;
	if (argc >= 2)	filename = argv[1];
	else            filename = defaultMeshFileName;

	// Initialize the OpenGL system
	GLFWwindow* window = initializeOpenGL(argc, argv, bgColor, true);
	if (window == NULL) return	-1;

	// Viewport and perspective setting
	reshapeModernOpenGL(window, windowW, windowH);

	// Callbacks
	glfwSetFramebufferSizeCallback(window, reshapeModernOpenGL);
	glfwSetKeyCallback(window, keyboard);

	// Depth test
	glEnable(GL_DEPTH_TEST);

	// Back face culling
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);

	// Vertex shader, fragment shader and program for Gouraud shading
	GLuint	vsGouraud = 0;
	GLuint	fsGouraud = 0;
	GLuint	programGouraud = 0;

	// Vertex shader, fragment shader and program for Phong shading
	GLuint	vsPhong = 0;
	GLuint	fsPhong = 0;
	GLuint	programPhong = 0;

	// VAO and VBO: a bunny in this casae
	GLuint	vao = 0;		// Vertex array object
	GLuint	indexId = 0;	// Vertex buffer object for triangle indices
	GLuint	vertexId = 0;	// Vertex buffer object for vertex positions
	GLuint	normalId = 0;	// Vertex buffer object for normal vectors

	int		numTris = 0;

	// Model matrix
	Matrix4f	ModelMatrix;
	ModelMatrix.setIdentity();

	// Initialization
	{
		// Create shaders, VAO and VBO for Gouraud and Phong
		createShaders("sv02_Gouraud.glsl", "sf02_Gouraud.glsl",
							programGouraud, vsGouraud, fsGouraud);
		createShaders("sv02_Phong.glsl", "sf02_Phong.glsl",
							programPhong, vsPhong, fsPhong);
		createVBO(vao, indexId, vertexId, normalId);

		// Load the mesh
		ArrayXXi	face;
		MatrixXf	vertex;
		MatrixXf	normal;

		cout << "Reading " << filename << endl;
		readMesh(filename, vertex, normal, face);

		// Upload the data into the buffers
		numTris = uploadMesh2VBO(face, vertex, normal, vao, indexId, vertexId, normalId);
	}

	// Usage
	cout << endl;
	cout << "Keyboard Input: space for play/pause" << endl;
	cout << "Keyboard Input: q/esc for quit" << endl;
	cout << endl;
	cout << "Keyboard Input: p for Phong/Gouraud shading" << endl;
	cout << "Keyboard Input: up/down to increase/decrease various coefficients" << endl;
	cout << "Keyboard Input: d for the diffuse coefficients" << endl;
	cout << "Keyboard Input: s for the specular coefficients" << endl;
	cout << "Keyboard Input: a for the ambient coefficients" << endl;
	cout << "Keyboard Input: n for the shininess coefficients" << endl;

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		// Update one frame if not paused
		if (!pause) update(ModelMatrix);

		// Draw one frame
		if (phong)	render(window, programPhong, vao, numTris, ModelMatrix);
		else        render(window, programGouraud, vao, numTris, ModelMatrix);

		glfwSwapBuffers(window);	// Swap buffers
		glfwPollEvents();			// Events
	}

	// Finalization
	{
		// Delete VBO and shaders
		deleteVBO(vao, indexId, vertexId, normalId);

		deleteShaders(programGouraud, vsGouraud, fsGouraud);
		deleteShaders(programPhong, vsPhong, fsPhong);
	}

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

void
update(Matrix4f& ModelMatrix)
{
	ModelMatrix.block<3, 3>(0, 0) = Matrix3f(AngleAxisf(0.02f * frame, Vector3f::UnitY()));

	frame++;
}

void
render(GLFWwindow* window, GLuint program,
		GLuint vao, int numTris, Matrix4f& ModelMatrix)
{
	// Antialiasing
	if (aaEnabled)	glEnable(GL_MULTISAMPLE);
	else            glDisable(GL_MULTISAMPLE);

	// Backgroud color
	glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Camera configuration
	ViewMatrix = lookAt<float>(eye, center, up);

	// ModelView matrix
	Matrix4f	ModelViewMatrix = ViewMatrix * ModelMatrix;
	setUniform(program, "ModelViewMatrix", ModelViewMatrix);

	// Normal matrix: Inverse of the transpose of the model view matrix
	Matrix3f	NormalMatrix = ModelViewMatrix.block<3,3>(0,0).inverse().transpose();
	setUniform(program, "NormalMatrix", NormalMatrix);

	// ModelViewProjection matrix in the vertex shader
	Matrix4f	ModelViewProjectionMatrix = ProjectionMatrix * ModelViewMatrix;
	setUniform(program, "ModelViewProjectionMatrix", ModelViewProjectionMatrix);

	// Light position in the fragment shader, represented in the view coordinate system
	Vector3f	l0 = ViewMatrix.block<3,3>(0,0) * light0 + ViewMatrix.block<3,1>(0,3);
	Vector3f	l1 = ViewMatrix.block<3, 3>(0, 0) * light1 + ViewMatrix.block<3, 1>(0, 3);
	setUniform(program, "Light0Position", l0);
	setUniform(program, "Light1Position", l1);

	// Draw objects: only the bunny in this case.
	{
		// Material is dependent of the object
		setUniform(program, "Ka", Ka);
		setUniform(program, "Kd", Kd);
		setUniform(program, "Ks", Ks);
		setUniform(program, "Shininess", Shininess);

		// Draw the mesh using the program and the vertex buffer object
		glUseProgram(program);
		drawVBO(vao, numTris);
	}

	// Check the status
	isOK("render()", __FILE__, __LINE__);
}

void
applyRangeCorrection(char command)
{
	for (int i = 0; i < 3; i++)
	{
		Kd[i] = max(Kd[i], 0.0f); Kd[i] = min(Kd[i], 1.0f);
		Ks[i] = max(Ks[i], 0.0f); Ks[i] = min(Ks[i], 1.0f);
		Ka[i] = max(Ka[i], 0.0f); Ka[i] = min(Ka[i], 1.0f);
	}
	Shininess = max(Shininess, 0.0f); Shininess = min(Shininess, 128.0f);

	switch (command)
	{
	case	'd': cout << "Kd = " << Kd.transpose() << endl; break;
	case	's': cout << "Ks = " << Ks.transpose() << endl; break;
	case	'a': cout << "Ka = " << Ka.transpose() << endl; break;
	case	'n': cout << "Shininess = " << Shininess << endl; break;
	}
}

void
increase()
{
	switch (command)
	{
	case	'd': Kd += Vector3f(0.1f, 0.1f, 0.1f);	break;
	case	's': Ks += Vector3f(0.1f, 0.1f, 0.1f);	break;
	case	'a': Ka += Vector3f(0.1f, 0.1f, 0.1f);	break;
	case	'n': Shininess += 1.0f;	break;
	}

	applyRangeCorrection(command);
}

void
decrease()
{
	switch (command)
	{
	case	'd': Kd -= Vector3f(0.1f, 0.1f, 0.1f);	break;
	case	's': Ks -= Vector3f(0.1f, 0.1f, 0.1f);	break;
	case	'a': Ka -= Vector3f(0.1f, 0.1f, 0.1f);	break;
	case	'n': Shininess -= 1.0f;	break;
	}

	applyRangeCorrection(command);
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

			// Gouraud/Phong
		case GLFW_KEY_P:
			phong = !phong;
			if (phong)	cout << "Phong shading" << endl;
			else        cout << "Gouraud shading" << endl;
			break;

			// Material properties
		case GLFW_KEY_D:	command = 'd'; break;	// Diffuse
		case GLFW_KEY_S:	command = 's'; break;	// Specular
		case GLFW_KEY_A:	command = 'a'; break;	// Ambient
		case GLFW_KEY_N:	command = 'n'; break;	// Shininess

		case GLFW_KEY_UP:	increase();	break;
		case GLFW_KEY_DOWN: decrease(); break;
		}
	}
}
