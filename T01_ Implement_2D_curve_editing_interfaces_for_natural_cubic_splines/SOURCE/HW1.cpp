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

// Picking
int		picked = -1;

vector<vector<float>> dataPoint;

//(x, y, z) of data points
int			N = -1;

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
	if (window == NULL) return	-1;

	// Callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetMouseButtonCallback(window, mouseButton);
	glfwSetCursorPosCallback(window, mouseMove);

	// Depth test
	glEnable(GL_DEPTH_TEST);

	// Viewport and perspective setting
	reshape(window, windowW, windowH);

	// Initialization - Main loop - Finalization
	init();

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		render(window);				// Draw one frame
		glfwSwapBuffers(window); 	// Swap buffers
		glfwPollEvents();			// Events
	}

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

// Linear system: Ac = b
//		from n equations of the form:
//		p_i(t) = c_0^i + (c_1^i * t^1) + (c_2^i * t^2) + (c_3^i * t^3)
//
MatrixXf	A;	// 4n x 4n matrix
MatrixXf	b;	// 4n x 3 to solve the 3 linear systems at once
MatrixXf	c;	// 4n x 3 to solve the 3 linear systems at once

void
buildLinearSystem()
{
	// Build A and b for N segments. A is independent of the locations of the points.
	A.resize(4 * N, 4 * N);
	A.setZero();

	b.resize(4 * N, 3);

	// Equation number
	int row = 0;

	// 2n equations for the end point interpolation
	for (int i = 0; i < N; i++, row += 2)
	{
		// p_i(0) = c_0^i
		A(row, 4 * i + 0) = 1;

		b(row, 0) = dataPoint[i][0];
		b(row, 1) = dataPoint[i][1];
		b(row, 2) = dataPoint[i][2];

		// p_i(1) = c_0^i + c_1^i + c_2^i + c_3^i
		A(row + 1, 4 * i + 0) = 1;
		A(row + 1, 4 * i + 1) = 1;
		A(row + 1, 4 * i + 2) = 1;
		A(row + 1, 4 * i + 3) = 1;

		b(row + 1, 0) = dataPoint[i + 1][0];
		b(row + 1, 1) = dataPoint[i + 1][1];
		b(row + 1, 2) = dataPoint[i + 1][2];
	}

	// (n-1) equations for the tangential continuity
	for (int i = 0; i < N - 1; i++, row++)
	{
		// p'_i(1) = 1*c_1^i + 2*c_2^i + 3*c_3^i = c_1^(i+1) = p'_(i+1)(0)
		A(row, 4 * i + 1) = 1;
		A(row, 4 * i + 2) = 2;
		A(row, 4 * i + 3) = 3;
		A(row, 4 * i + 5) = -1;

		b(row, 0) = 0;
		b(row, 1) = 0;
		b(row, 2) = 0;
	}

	// (n-1) equations for the second-derivative continuity
	for (int i = 0; i < N - 1; i++, row++)
	{
		// p''_i(1) = 2*c_2^i + 6*c_3^i = 2*c_2^(i+1) = p''_(i+1)(0)
		A(row, 4 * i + 2) = 2;
		A(row, 4 * i + 3) = 6;
		A(row, 4 * i + 6) = -2;

		b(row, 0) = 0;
		b(row, 1) = 0;
		b(row, 2) = 0;
	}

	// 2 equations for the natural boundary condition
	{
		// p''_0(0) = 2*c_2^0 = 0
		A(row, 2) = 2;

		b(row, 0) = 0;
		b(row, 1) = 0;
		b(row, 2) = 0;

		row++;

		// p''_(n-1)(1) = 2*c_2^(n-1) + 6*c_3^(n-1) = 0
		A(row, 4 * (N - 1) + 2) = 2;
		A(row, 4 * (N - 1) + 3) = 6;

		b(row, 0) = 0;
		b(row, 1) = 0;
		b(row, 2) = 0;

		row++;
	}
}

void
solveLinearSystem()
{
	// Solve the 3 linear systems at once
	c = A.colPivHouseholderQr().solve(b);
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

Vector3f
pointOnNaturalCubicSplineCurve(int i, float t)
{
	float	x = c(4 * i + 0, 0) + (c(4 * i + 1, 0) + (c(4 * i + 2, 0) + c(4 * i + 3, 0) * t) * t) * t;
	float	y = c(4 * i + 0, 1) + (c(4 * i + 1, 1) + (c(4 * i + 2, 1) + c(4 * i + 3, 1) * t) * t) * t;
	float	z = c(4 * i + 0, 2) + (c(4 * i + 1, 2) + (c(4 * i + 2, 2) + c(4 * i + 3, 2) * t) * t) * t;
	
	return	Vector3f(x, y, z);
}

// Draw the natural cubic spline
void
drawNatrualCubicSpline()
{
	// Curve
	glLineWidth(1.5 * dpiScaling);
	glColor3f(0, 0, 0);
	for (int i = 0; i < N; i++)
	{
		// N_SUB_SEGMENTS for each curve segment
		glBegin(GL_LINE_STRIP);
		for (int j = 0; j <= N_SUB_SEGMENTS; j++)
		{
			float		t = (float)j / N_SUB_SEGMENTS;	// [0, 1]
			Vector3f	p = pointOnNaturalCubicSplineCurve(i, t);

			glVertex3fv(p.data());
		}
		glEnd();
	}

	// Sample points at the curve
	if (sampledPointsEnabled)
	{
		glPointSize(5 * dpiScaling);
		glColor3f(0, 0, 0);
		glBegin(GL_POINTS);
		for (int i = 0; i < N; i++)
		{
			// N_SUB_SEGEMENTS for each curve segment
			for (int j = 1; j < N_SUB_SEGMENTS; j++)
			{
				float		t = (float)j / N_SUB_SEGMENTS;
				Vector3f	p = pointOnNaturalCubicSplineCurve(i, t);

				glVertex3fv(p.data());
			}
		}
		glEnd();
	}
}

void
drawDataPoint()
{
	glPushName(-1);
	// Data points
	glPointSize(10 * dpiScaling);
	glColor3f(0, 0, 1);
	for (int i = 0; i < N+1; i++)
	{
		glLoadName(i);

		glBegin(GL_POINTS);
		glVertex3f(dataPoint[i][0], dataPoint[i][1], dataPoint[i][2]);
		glEnd();
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

	drawDataPoint();
	// Draw the natural cubic spline curve
	if (N > 0)
		drawNatrualCubicSpline();
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

		case GLFW_KEY_SPACE: 
			cout << "Data Point Count is "  << N + 1 << endl;
			for (int i = 0; i < N + 1; i++)
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

	for (int i = 0; i < N + 1; i++)
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
	float		subNum = -1;
	float		nearest = range;
	float		length = -1;
	Vector3f	p(xw, yw, 0);

	for (int i = 0; i < N; i++)
	{
		// N_SUB_SEGMENTS for each curve segment
		for (int j = 0; j < N_SUB_SEGMENTS; j++)
		{
			float		t = (float)j / N_SUB_SEGMENTS;	// [0, 1]
			Vector3f	p1 = pointOnNaturalCubicSplineCurve(i, t);

			float		t1 = (float)(j + 1) / N_SUB_SEGMENTS;
			Vector3f	p2 = pointOnNaturalCubicSplineCurve(i, t1);

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
					subNum = t;
					length = distancep2e;
				}
			}
		}
	}

	//cout << "This is Curve Segment Number " << index[0] << endl;
	//cout << "This is N Sub Segment Number " << index[1] << endl;
	//cout << "This is Nearest " << nearest << endl;
	//cout << endl;

	if (length > -1)
	{
		vector<float> v;
		dataPoint.insert(dataPoint.begin() + curveNum + 1, v);

		Vector3f	p1 = pointOnNaturalCubicSplineCurve(curveNum, subNum);
		Vector3f	p2 = pointOnNaturalCubicSplineCurve(curveNum, subNum + ((float)1 / N_SUB_SEGMENTS));
		Vector3f	foot = p1 + (p2 - p1).normalized() * length;

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
			N += 1;
			vector<float> v;
			dataPoint.push_back(v);
			dataPoint[N].push_back(xw);
			dataPoint[N].push_back(yw);
			dataPoint[N].push_back(0);
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

		if (N > 0)
		{
			buildLinearSystem();
			solveLinearSystem();
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

			if (N > 0)
			{
				buildLinearSystem();
				solveLinearSystem();
			}
		}
	}
}