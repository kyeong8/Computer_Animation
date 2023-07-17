#include "glSetup.h"
#include "hsv2rgb.h"

#ifdef  _WIN32
#define _USE_MATH_DEFINES	// To include the definition of M_PI in math.h
#endif

#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>
#include <vector>
using namespace std;

void	init();
void	render(GLFWwindow* window);
void	reshape(GLFWwindow* window, int w, int h);
void	keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);
void	mouseButton(GLFWwindow* window, int button, int action, int mods);
void	mouseMove(GLFWwindow* window, double x, double y);

// Colors
GLfloat bgColor[4] = { 1, 1, 1, 1 };

// Controls
bool	sampledPointsEnabled = false;
int		N_SUB_SEGMENTS = 10;

bool ctrlPolygonDrawingEnabled = false;
int iSegment = 0;

// Picking
int		picked = -1;

vector<vector<float>> dataPoint;

//(x, y, z) of data points
int			N = 0;

float		range = 0.02;

// Picking
enum PickMode {
	NONE = 0, ADD = 1, REMOVE = 2, DRAGGING = 3, COMPLETE = 4, INSERT = 5,
};	PickMode pickMode = NONE;

string PickModeName[6] = { "NONE", "ADD", "REMOVE", "DRAG", "COMPLETE", "INSERT"};

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
	glfwSetMouseButtonCallback(window, mouseButton);
	glfwSetCursorPosCallback(window, mouseMove);

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

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

void
init()
{
	// Usage
	cout << endl;
	cout << "Keyboard Input: s for sampled points on/ off" << endl;
	cout << "Keyboard Input: up/down to increase/decrease the number of samples" << endl;
	cout << "Keyboard Input: a for Add data points" << endl;
	cout << "Keyboard Input: r for Select/remove data points" << endl;
	cout << "Keyboard Input: d for Select/drag data points" << endl;
	cout << "Keyboard Input: i for Select edges and insert 2 data points" << endl;

	cout << "Keyboard Input: q/esc for quit" << endl;
	cout << endl;
}

// Compute the point on the B-spline
Vector3f
pointOnBspline(const Vector3f b[4], float t1)
{
	float t2 = t1 * t1;
	float t3 = t2 * t1;

	float B0 = 1 - 3 * t1 + 3 * t2 - t3;
	float B1 = 4 - 6 * t2 + 3 * t3;
	float B2 = 1 + 3 * t1 + 3 * t2 - 3 * t3;
	float B3 = t3;

	return (b[0] * B0 + b[1] * B1 + b[2] * B2 + b[3] * B3) / 6;
}

void
drawControlPolygon()
{
	glLineWidth(1.5f * dpiScaling);
	Vector3f b[4];

	glColor3f(0.5f, 0.5f, 0.5f);

	glEnable(GL_LINE_STIPPLE);
	glLineStipple(int(3 * dpiScaling), 0xcccc);

	for (int j = 0; j < 4; j++)
		b[j] = Vector3f(dataPoint[iSegment + j][0], dataPoint[iSegment + j][1], 0);

	// N_SUB_SEGMENTS for each curve segment
	glBegin(GL_LINE_STRIP);
	for (int j = 0; j < 4; j++)
		glVertex3fv(b[j].data());
	glEnd();

	glDisable(GL_LINE_STIPPLE);
}

// Draw the uniform cubic B - spline curve
void
drawBSpline()
{
	// Colors
	float hsv[3] = { 0,1,1 };	// [0,360], [0,1], [0,1]
	float rgb[3];

	// Curve
	Vector3f b[4];
	for (int i = 0; i < N - 3; i++)
	{
		// To make an alternating complementary color
		hsv[0] = 180.0f * i / (N - 3) + ((i % 2) ? 180.0f : 0);
		HSV2RGB(hsv, rgb);
		glColor3f(rgb[0], rgb[1], rgb[2]);

		if (ctrlPolygonDrawingEnabled && i == iSegment)
		 	 glLineWidth(3 * dpiScaling);
		else glLineWidth(1.5f * dpiScaling);

		for (int j = 0; j < 4; j++)
			b[j] = Vector3f(dataPoint[i + j][0], dataPoint[i + j][1], 0);

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
	if (sampledPointsEnabled)
	{
		glPointSize(5 * dpiScaling);

		for (int i = 0; i < N - 3; i++)
		{
			hsv[0] = 180.0f * i / (N - 3) + ((i % 2) ? 180.0f : 0);
			HSV2RGB(hsv, rgb);
			glColor3f(rgb[0], rgb[1], rgb[2]);

			for (int j = 0; j < 4; j++)
				b[j] = Vector3f(dataPoint[i + j][0], dataPoint[i + j][1], 0);

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
	for (int i = 0; i < N; i++)
	{
		glVertex3f(dataPoint[i][0], dataPoint[i][1], dataPoint[i][2]);
	}
	glEnd();

	// Control polygon
	if (ctrlPolygonDrawingEnabled) drawControlPolygon();
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
	if ((action == GLFW_PRESS || action == GLFW_REPEAT) && !(mods & GLFW_MOD_CONTROL))
	{
		switch (key)
		{
			// Sampled points
		case GLFW_KEY_S:	sampledPointsEnabled = !sampledPointsEnabled;	break;

		// Number of smaples
		case GLFW_KEY_UP:	N_SUB_SEGMENTS++;	break;
		case GLFW_KEY_DOWN:	N_SUB_SEGMENTS = max(N_SUB_SEGMENTS-1, 1);	break;

			// Quit
		case GLFW_KEY_Q:
		case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GL_TRUE); break;

			// Add 10 data points
		case GLFW_KEY_A:	cout << "Change ADD Mode\n" << endl; pickMode = ADD;	break;

			//  Select/remove 3 data points
		case GLFW_KEY_R:	cout << "Change REMOVE Mode\n" << endl; pickMode = REMOVE;	break;

			//  Select/drag 2 data points
		case GLFW_KEY_D:	cout << "Change DRAG Mode\n" << endl; pickMode = COMPLETE;	break;

			// Select edges and insert 2 data points
		case GLFW_KEY_I:	cout << "Change INSERT Mode\n" << endl; pickMode = INSERT;	break;

			// Control polygon
		case GLFW_KEY_C:	ctrlPolygonDrawingEnabled = !ctrlPolygonDrawingEnabled; break;

		case GLFW_KEY_LEFT: iSegment = max(iSegment - 1, 0); break;
		case GLFW_KEY_RIGHT:iSegment = min(iSegment + 1, N - 4); break;

		case GLFW_KEY_SPACE: 
			cout << "Data Point Count is "  << N << endl;
			for (int i = 0; i < N; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					cout << dataPoint[i][j] << endl;
				}
				cout << endl;
			}
		}
	}

	if ((action == GLFW_PRESS || action == GLFW_REPEAT) && (mods & GLFW_MOD_CONTROL))
	{
		switch (key)
		{
			// Navigation
		case GLFW_KEY_UP:		 range += 0.1f; cout << "range is " << range << endl; 	break;
		case GLFW_KEY_DOWN:		 range -= 0.1f; cout << "range is " << range << endl; 	break;
		}
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
findNearestPoint(float xw, float yw)
{
	float nearest = range;
	int name = -1;

	for (int i = 0; i < N; i++)
	{
		Vector3f p1(xw, yw, 0);
		VectorXf p2 = toEigenVector(dataPoint[i]);

		Vector3f dist = p2 - p1;
		float distance = dist.norm();

		if (nearest > distance)
		{
			nearest = distance;
			name = i;
		}
	}

	if (name > -1 && pickMode == REMOVE)
	{
		N -= 1;
		dataPoint.erase(dataPoint.begin() + name);
	}

	if (name > -1 && pickMode == DRAGGING)
	{
		picked = name;
	}
}

void
findNearestEdge(float xw, float yw)
{
	int			curveNum = -1;
	float		nearest = range;
	float		length = -1;
	Vector3f	p(xw, yw, 0);

	for (int i = 0; i < N-1; i++)
	{
		Vector3f p1;
		Vector3f p2;
		p1 = Vector3f(dataPoint[i][0], dataPoint[i][1], 0);
		p2 = Vector3f(dataPoint[i + 1][0], dataPoint[i + 1][1], 0);

		Vector3f v1 = p2 - p1;
		float distancep2p = v1.norm();

		Vector3f v2 = p - p1;
		float distancep2e = v1.dot(v2);
		distancep2e /= distancep2p;

		v1.normalize();
		v2.normalize();

		Vector3f	axis = v1.cross(v2);
		float		sinTheta = axis.norm();
		float		cosTheta = v1.dot(v2);
		float		theta = (float)atan2(sinTheta, cosTheta) * float(180.0 / M_PI);

		//cout << i << "번째 Curve Segment, " << j <<"번째 Sub Segment"<< endl;
		//cout << "Theta is " << theta << endl;
		//cout << "This is Distance Point to Edge " << distancep2e << endl;
		//cout << "This is Distance Point to Point " << distancep2p << endl;
		//cout << endl;

		if (theta < 90 && distancep2e < distancep2p)
		{
			Vector3f v = p - p1;
			float distance = sinTheta * v.norm();
			if (nearest > distance)
			{
				nearest = distance;
				curveNum = i;
				length = distancep2e;
			}
		}
	}

	//cout << "This is Curve Segment Number " << index[0] << endl;
	//cout << "This is N Sub Segment Number " << index[1] << endl;
	//cout << "This is Nearest " << nearest << endl;
	//cout << endl;

	if (length > -1)
	{
		Vector3f	p1 = Vector3f(dataPoint[curveNum][0], dataPoint[curveNum][1], 0);
		Vector3f	p2 = Vector3f(dataPoint[curveNum + 1][0], dataPoint[curveNum + 1][1], 0);
		Vector3f	foot = p1 + (p2 - p1).normalized() * length;

		vector<float> v;
		dataPoint.insert(dataPoint.begin() + curveNum + 1, v);

		dataPoint[curveNum + 1].push_back(foot[0]);
		dataPoint[curveNum + 1].push_back(foot[1]);
		dataPoint[curveNum + 1].push_back(foot[2]);

		N += 1;
	}
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
	double x = xs * dpiScaling;	//	for 
	double y = ys * dpiScaling;	//	for HiDPI

	float xw, yw;
	screen2world(xs, ys, xw, yw);

	if (action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (pickMode == COMPLETE)
			pickMode = DRAGGING;

		if (pickMode == ADD)
		{
			vector<float> v;
			dataPoint.push_back(v);
			dataPoint[N].push_back(xw);
			dataPoint[N].push_back(yw);
			dataPoint[N].push_back(0);
			N += 1;
			///sort(dataPoint.begin(), dataPoint.end());
		}

		else if (pickMode == REMOVE || pickMode == DRAGGING)
		{
			findNearestPoint(xw, yw);
		}
		else if (pickMode == INSERT && N > 0)
		{
			findNearestEdge(xw, yw);
		}
	}

	if (action == GLFW_RELEASE && button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (pickMode == DRAGGING)
		{
			pickMode = COMPLETE;
			picked = -1;
		}
	}
}

void
mouseMove(GLFWwindow* window, double xs, double ys)
{
	float xw, yw;

	// Update the end point while dragging
	if (pickMode == DRAGGING)
	{
		//cout << picked << endl;
		//cout << endl;

		if (picked > -1)
		{
			screen2world(xs, ys, xw, yw);
			dataPoint[picked][0] = xw;
			dataPoint[picked][1] = yw;
		}
	}
}