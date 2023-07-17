
#include "glSetup.h"
#include "hsv2rgb.h"

#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>
using namespace std;

void init();
void render(GLFWwindow* window);
void reshape(GLFWwindow* window, int w, int h);
void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);

// Endpoint interpolation
void prepareRepeatedControlPoints(int repetition);
void deleteRepeatedControlPoints();

// Colors
GLfloat bgColor[4] = { 1,1,1,1 };

// Controls
bool samplePointDrawingEnabled = false;
int N_SUB_SEGMENTS = 10;

bool ctrlPolygonDrawingEnabled = false;
int iSegment = 0;

int 
main(int argc, char* argv[])
{
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

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		render(window);				// Draw one frame
		glfwSwapBuffers(window);	// Swap buffers
		glfwPollEvents();			// Events
	}

	// Finalization
	deleteRepeatedControlPoints();

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}


//(x,y,z) positions of the control points
const int N = 8; // Here, N is the number of control points
const float p[N][3] = {
	{-1.6f,0.8f,0},
	{-1.2f,-0.8f,0},
	{-0.8f,0.4f,0},
	{-0.4f,0.8f,0},
	{0.4f,-0.4f,0},
	{0.8f,0.8f,0},
	{1.2f,-0.8f,0},
	{1.6f,0.8f,0},
};

// Including the repeated control points
int		  nControlPoints = 0;
Vector3f* controlPoints = NULL;

// repetition = 2 for endpoint interpolation
void 
prepareRepeatedControlPoints(int repetition)
{
	deleteRepeatedControlPoints();

	nControlPoints = N + 2 * repetition;
	controlPoints = new Vector3f[nControlPoints];

	for (int i = 0; i < N; i++)
		controlPoints[i + repetition] = Vector3f(p[i][0], p[i][1], p[i][2]);

	for (int i = 0; i < repetition; i++)
	{
		controlPoints[i] = controlPoints[repetition];							// From begining
		controlPoints[N + repetition + i] = controlPoints[N + repetition - 1];	// Before ending
	}
}

void deleteRepeatedControlPoints()
{
	if (controlPoints) delete[] controlPoints;
}

void 
init()
{
	// Initially, no repetition
	prepareRepeatedControlPoints(0);

	// Usage
	cout << endl;
	cout << "Keyboard Input: [0,2] for # of repetitions in the endpoint interpolation" << endl;
	cout << "Keyboard Input: s for drawing sample points on/off" << endl;
	cout << "Keyboard Input: up/down to increase/decrease the number of samples" << endl;
	cout << "Keyboard Input: c for drawing control polygon on/off" << endl;
	cout << "Keyboard Input: left/right to move the control polygon" << endl;
	cout << "Keyboard Input: q/esc for quit" << endl;
	
}

// Compute the point on the B-spline
Vector3f 
pointOnBspline(const Vector3f b[4],float t1)
{
	float t2 = t1 * t1;
	float t3 = t2 * t1;

	float B0 = 1 - 3 * t1 + 3 * t2 - t3;
	float B1 = 4 - 6 * t2 + 3 * t3;
	float B2 = 1 + 3 * t1 + 3 * t2 - 3 * t3;
	float B3 = t3;
	
	return (b[0] * B0 + b[1] * B1 + b[2] * B2 + b[3] * B3 ) / 6;
}

// Draw the uniform cubic B-spline curve
void 
drawBSpline()
{
	// Colors
	float hsv[3] = { 0,1,1 };	// [0,360], [0,1], [0,1]
	float rgb[3];

	// Curve
	Vector3f b[4];
	for (int i = 0; i < nControlPoints - 3; i++)
	{
		// To make an alternating complementary color
		hsv[0] = 180.0f * i / (nControlPoints - 3) + ((i % 2) ? 180.0f : 0);
		HSV2RGB(hsv, rgb);
		glColor3f(rgb[0], rgb[1], rgb[2]);
		
		if (ctrlPolygonDrawingEnabled && i == iSegment)
			glLineWidth(3 * dpiScaling);
		else  glLineWidth(1.5f * dpiScaling);
	
		for (int j = 0; j < 4; j++)
			b[j] = controlPoints[i + j];

		// N_SUB_SEGMENTS for each curve segment
		glBegin(GL_LINE_STRIP);
		for (int j = 0; j <= N_SUB_SEGMENTS; j++)
		{
			float t = (float)j / N_SUB_SEGMENTS;	// [0, 1]
			Vector3f pt = pointOnBspline(b, t);

			glVertex3fv(pt.data());
		}
		glEnd();
	}
	
	// Sample points at the curve
	if (samplePointDrawingEnabled)
	{
		glPointSize(5 * dpiScaling);

		for (int i = 0; i < nControlPoints-3; i++)
		{
			hsv[0] = 180.0f * i / (nControlPoints - 3) + ((i % 2) ? 180.0f : 0);
			HSV2RGB(hsv, rgb);
			glColor3f(rgb[0], rgb[1], rgb[2]);

			for (int j = 0; j < 4; j++)
				b[j] = controlPoints[i + j];

			// N_SUB_SEGMENTS for each curve segment
			glBegin(GL_POINTS);
			for (int j = 0; j <= N_SUB_SEGMENTS; j++)
			{
				float t = (float)j / N_SUB_SEGMENTS;	// [0, 1]
				Vector3f pt = pointOnBspline(b, t);

				glVertex3fv(pt.data());
			}
			glEnd();
		}
	}

	// Control points
	glPointSize(10 * dpiScaling);
	glColor3f(1, 0, 0);
	glBegin(GL_POINTS);
	for (int i = 0; i < N + 1; i++)
	{
		glVertex3f(p[i][0], p[i][1], p[i][2]);
	}
	glEnd();

	// Control polygon
	// void drawControlPolygon();
	// if (ctrlPolygonDrawingEnabled) drawControlPolygon();
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

	// Draw the natural cubic spline curve
	drawBSpline();
}

void 
keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT)
	{
		switch (key)
		{
			// Sampled points
		case GLFW_KEY_S: samplePointDrawingEnabled = !samplePointDrawingEnabled; break;

			// Number of samples
		case GLFW_KEY_UP: N_SUB_SEGMENTS++; break;
		case GLFW_KEY_DOWN: N_SUB_SEGMENTS = max(N_SUB_SEGMENTS - 1, 1); break;

			// Reptition
		case GLFW_KEY_0: prepareRepeatedControlPoints(0); break;
		case GLFW_KEY_1: prepareRepeatedControlPoints(1); break;
		case GLFW_KEY_2: prepareRepeatedControlPoints(2); break;
			
			// Quit
		case GLFW_KEY_Q:
		case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GL_TRUE); break;
		
			// Control polygon
		case GLFW_KEY_C:	ctrlPolygonDrawingEnabled = !ctrlPolygonDrawingEnabled; break;

			// Control polygon
		case GLFW_KEY_LEFT: iSegment = max(iSegment - 1, 0); break;
		case GLFW_KEY_RIGHT:iSegment = min(iSegment + 1, nControlPoints - 4); break;
		}
	}
}