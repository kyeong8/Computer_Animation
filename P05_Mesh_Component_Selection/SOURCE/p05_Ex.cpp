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

#include <vector>
#include <unordered_map>
#include <map>
#include <queue>	// Breadth-first traversal

#undef NDEBUG
#include <assert.h>

void	init(const char* filename);
void	setupLight(const Vector4f& position);
void	setupColoredMaterial(const Vector3f& color);
void	render(GLFWwindow* window, bool selectionMode);
void	reshape(GLFWwindow* window, int w, int h);
void	keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);
void	mouseButton(GLFWwindow* window, int button, int action, int mods);
void	mouseMove(GLFWwindow* window, double x, double y);
void	reset();

// Camera configuration
Vector3f	eye(2, 2, 2);
Vector3f	center(0, 0.2f, 0);
Vector3f	up(0, 1, 0);

// Light configuration
Vector4f	light(5.0, 5.0, 0.0, 1);		// Light position

// Play configuration
bool	pause = true;
float	timeStep = 1.0f / 120;		// 120fps
float	period = 8.0;
float	theta = 0;

// Global coordinate frame
bool	axes = false;
float	AXIS_LENGTH = 1.5f;
float	AXIS_LINE_WIDTH = 2;

// Colors
GLfloat bgColor[4] = { 1, 1, 1, 1 };

// Default mesh file name
const char* defaultMeshFileName = "m01_bunny.off";

// Display style
bool	aaEnabled = true;	// Antialiasing
bool	bfcEnabled = true;	// Back face culling

// Mesh
MatrixXf	vertex;			// Position
MatrixXf	faceNormal;		// Face normal vector
MatrixXf	vertexNormal;	// Vertex normal vector
ArrayXXi	face;			// Index

// Mesh that consists of faces with gap
MatrixXf	faceVertex;

bool	faceWithGapMesh = true;
bool	useFaceNormal = true;
float	gap = 0.3f;

bool	diagnosis = false;

// Picking
enum PickMode {
	NONE = 0, VERTEX = 1, EDGE = 2, FACE = 3,
};	PickMode pickMode = FACE;

int						picked = -1;
unordered_map<int, int> pickedNeighborVertices;	// {vertex, n-ring}
unordered_map<int, int> pickedNeighborEdges;	// {edge, n-ring}
unordered_map<int, int> pickedNeighborFaces;	// {face, n-ring}

bool	vertexAdjacent = true;

// Mouse input mode
enum class InputMode
{
	NONE = 0,
	DRAGGING = 1,
	COMPLETE = 2,
};

InputMode	inputMode = InputMode::NONE;

double point[2][2] = { {0, 0}, {0, 0} };

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
	glfwSetMouseButtonCallback(window, mouseButton);
	glfwSetCursorPosCallback(window, mouseMove);

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
			if (!pause) theta += float(2.0 * M_PI) / period * elapsed;

			elapsed = 0;	// Reset the elapsed time
		}

		render(window, false);		// Draw one frame
		glfwSwapBuffers(window); 	// Swap buffers
	}

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

// Data structures for mesh traversal
//
int nEdges = 0;

// Eijf[i] = {j, f}
// The vertex i has an edge (i, j) and the face f is adjacent to (i, j) from the right.
//
vector<unordered_map<int, int>> Eijf;	// Eijf[i] = {j, f}

// E[e] = {start, end}
// The edge e has the start vertex and the end vertex.
//
vector<pair<int, int>>			E;		// E[e] = {start, end}

// Edges adjacent to a vertex: edges sharing the vertex
//
// Ev[v][j] = e
// e is the j-the edge sharing the vertex v
//
vector<vector<int>> Ev;		// Ev[v][j] = e

// Faces adjacent to a vertex: faces sharing the vertex
//
// Fv[v][j] = f
// f is the j-th face sharing the vertex v.
//
vector<vector<int>> Fv;		// Fv[v][f] = f

// Distance from the picked face, edge, and vertex
vector<int> Dv, De, Df;

// N-ring neighborhood
int nRing = 0;

// Data structures for mesh traversal
void
prepareMeshTraversal()
{
	// Directed edge Eijf[i] = {j, f}.
	// The vertex i has an edge (i, j) and the face f is adjacent to (i, j) from the right.
	//
	Eijf.resize(vertex.cols());
	Fv.resize(vertex.cols());
	for (int f = 0; f < face.cols(); f++)
	{
		// The vertex indices are in CCW. Thus, f become the right face in CW.
		for (int i = 0; i < 3; i++)
			Eijf[face((i + 1) % 3, f)].insert({ face(i, f), f });

		// Faces adjacent to a vertex: faces sharing the vertex
		for (int i = 0; i < 3; i++)
			Fv[face(i, f)].push_back(f);
	}

	// Undirected edge E[e] = {start, end} s.t. start < end
	// The edge e has the start vertex and the end vertex.
	//
	Ev.resize(vertex.cols());
	for (int i = 0; i < vertex.cols(); i++)
	{
		// For all the directed edges (i, j)
		for (auto& e : Eijf[i])
		{
			int j = e.first;	// End vertex

			if (i < j)	// Just add this edge
			{
				E.push_back({ i, j });		// i < j 
			}
			else   {
				// Check to see if there exists the edge (j, i)
				unordered_map<int, int>::const_iterator e = Eijf[j].find(i);

				// If exists, skip this directed edge. It will be added later.
				if (e != Eijf[j].end()) continue;

				// Otherwise, add this edge at this time. This is a boundary edge.
				E.push_back({ j, i });			// j < i
			}

			// E[e] = {i, j}: The edge e has the start vertex i and the end vertex j.
			int e_id = int(E.size()) - 1;

			Ev[i].push_back(e_id);
			Ev[j].push_back(e_id);
		}
	}
	nEdges = int(E.size());
	cout << "# undirected edges = " << nEdges << endl;

	// Initialize distances from the selected entities
	Df.resize(face.cols());
	for (int i = 0; i < face.cols(); i++)
		Df[i] = -1;

	De.resize(nEdges);
	for (int i = 0; i < nEdges; i++)
		De[i] = -1;

	Dv.resize(vertex.cols());
	for (int i = 0; i < vertex.cols(); i++)
		Dv[i] = -1;
}

// Find all the vertices adjacent to the seed vertex
void
findNeighborVertices(int seedVertex, int nRing, unordered_map<int, int>& neighborVertices)
{
	Dv[seedVertex] = 0;

	queue<int>	Q;			// Queue for breadth-first search
	Q.push(seedVertex);		// The seed edge to be visited
	for (; !Q.empty(); Q.pop())
	{
		// Dealing with the next vertex with the distance Dv[v] from the seed vertex
		int v = Q.front();

		// Obtain the edge adjacent to the vertex v
		for (int e : Ev[v])
		{
			int a[2] = { E[e].first, E[e].second };
			for (int i = 0; i < 2; i++)
			{
				// Check to see if the adjacent vertex a[i] is already visited
				if (Dv[a[i]] != -1)	continue;

				Dv[a[i]] = Dv[v] + 1;		// Distance
				neighborVertices.insert({ a[i], Dv[a[i]] });

				// Check to see the desired level is reached
				if (Dv[a[i]] >= nRing)	continue;

				// Insert the adjacent edge a to the queue
				Q.push(a[i]);
			}
		}
	}
}

// Find all the edges adjacent to the seed edge
void
findNeighborEdges(int seedEdge, int nRing, unordered_map<int, int>& neighborEdges)
{
	De[seedEdge] = 0;

	queue<int>	Q;			// Queue for breadth-first search
	Q.push(seedEdge);		// The seed edge to be visited
	for (; !Q.empty(); Q.pop())
	{
		// Dealing with the next edge with the distance De[e] from the seed edge
		int e = Q.front();

		int v[2] = { E[e].first, E[e].second };
		for (int i = 0; i < 2; i++)
		{
			// Obtain the edge adjacent to the vertex v[i]
			for (int a: Ev[v[i]])
			{
				// Check to see if the adjacent edge a is already visited
				if (De[a] != -1)	continue;

				De[a] = De[e] + 1;		// Distance
				neighborEdges.insert({ a, De[a] });

				// Check to see the desired level is reached
				if (De[a] >= nRing)	continue;

				// Insert the adjacent edge a to the queue
				Q.push(a);
			}
		}
	}
}


// Find the face adjacent to the edge (i, j) from the right
int
findEdgeAdjacentFace(int i, int j)
{
	// Find the edge (i, j) which is already inserted
	unordered_map<int, int>::const_iterator e = Eijf[i].find(j);
	if (e == Eijf[i].end())	return	-1;

	return e->second;	// e-first = j and e->second is the face id
}

// Find all the faces adjacent to the seed face at an EDGE
void
findEdgeAdjacentFaces(int seedFace, int nRing, unordered_map<int, int>& neighborFaces)
{
	queue<int>	Q;			// Queue for breadth-first search
	Q.push(seedFace);		// The seed face to be visited
	for (; !Q.empty(); Q.pop())
	{
		// Dealing with the next face with the distance Df[f] from the seed face
		int f = Q.front();

		for (int i = 0; i < 3; i++)
		{
			// Obtain the face adjacent to the edge (i, i+1)
			int a = findEdgeAdjacentFace(face(i, f), face((i+1)%3, f));
			if (a == -1)	continue;

			// Check to see if the adjacent face a is already visited
			if (Df[a] != -1)	continue;

			Df[a] = Df[f] + 1;		// Distance
			neighborFaces.insert({ a, Df[a] });

			// Check to see the desired level is reached
			if (Df[a] >= nRing)	continue;

			// Insert the adjacent edge a to the queue
			Q.push(a);
		}
	}
}

// Find all the faces adjacent to the seed face at an VERTEX
void
findVertexAdjacentFaces(int seedFace, int nRing, unordered_map<int, int>& neighborFaces)
{

	queue<int>	Q;			// Queue for breadth-first search
	Q.push(seedFace);		// The seed face to be visited
	for (; !Q.empty(); Q.pop())
	{
		// Dealing with the next face with the distance Df[f] from the seed face
		int f = Q.front();

		for (int i = 0; i < 3; i++)
		{
			// Obtain the face adjacent to the vertex i
			for (int a : Fv[face(i, f)])
			{
				// Check to see if the adjacent face a is already visited
				if (Df[a] != -1)	continue;

				Df[a] = Df[f] + 1;		// Distance
				neighborFaces.insert({ a, Df[a] });

				// Check to see the desired level is reached
				if (Df[a] >= nRing)	continue;

				// Insert the adjacent edge a to the queue
				Q.push(a);
			}
		}
	}
}

// Find all the faces adjacent to the seed face
void
findNeighborFaces(int seedFace, int nRing, unordered_map<int, int>& neighborFaces)
{
	if (vertexAdjacent) findVertexAdjacentFaces(picked, nRing, neighborFaces);
	else                findEdgeAdjacentFaces(picked, nRing, neighborFaces);
}

// Build shrunken faces
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
	nEdges = readMesh(filename, vertex, face, faceNormal, vertexNormal);
	cout << "# undirected edges = " << nEdges << endl;

	// Shrunken face mesh
	buildShrunkenFaces(vertex, faceVertex);

	// Prepare data structures for the mesh traversal
	prepareMeshTraversal();

	// Usage
	cout << endl;
	cout << "Keyboard Input: space for play/pause" << endl;
	cout << "Keyboard Input: x for axes on/off" << endl;
	cout << "Keyboard Input: q/esc for quit" << endl;
	cout << endl;
	cout << "Keyboard Input: v for vertex selection" << endl;
	cout << "Keyboard Input: e for edge selection" << endl;
	cout << "Keyboard Input: f for face selection" << endl;
	cout << "Keyboard Input: g for face with gap mesh/mesh" << endl;
	cout << "Keyboard Input: up/down to increase/decrease the gap between faces" << endl;
	cout << "Keyboard Input: [0:5] for n-ring" << endl;
	cout << "Keyboard Input: a for vertex/Edge adjacency" << endl;
	cout << "Keyboard Input: b for back face culling on/off" << endl;
}

Vector3f	selectionColor[6] =
{
	{1, 0, 0},	// Red
	{1, 1, 0},	// Yellow
	{0, 1, 0},	// Green
	{0, 1, 1},	// Cyan
	{0, 0, 1},	// Blue
	{1, 0, 1}	// Magenta
};

void
screen2world(float xs, float ys, float& xw, float& yw)
{
	// In the world space. See reshape() in glSetup.cpp
	float	aspect = (float)screenW / screenH;
	xw = 2.0f * (xs / (screenW - 1) - 0.5f) * aspect;
	yw = -2.0f * (ys / (screenH - 1) - 0.5f);
}

void
drawRectangle()
{
	float x0, y0, x1, y1;
	screen2world((float)point[0][0], (float)point[0][1], x0, y0);
	screen2world((float)point[1][0], (float)point[1][1], x1, y1);

	// Attribute of the line 
	glLineWidth(5 * dpiScaling);

	glEnable(GL_LINE_STIPPLE);
	glLineStipple(int(3 * dpiScaling), 0xcccc);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	perspectiveView = false;
	setupProjectionMatrix();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glBegin(GL_LINE_LOOP);
	glVertex2f(x0, y0);
	glVertex2f(x1, y0);
	glVertex2f(x1, y1);
	glVertex2f(x0, y1);
	glEnd();

	perspectiveView = true;

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glDisable(GL_LINE_STIPPLE);
}

// Draw a sphere after setting up its material
void
drawFaces()
{
	// Picking
	glPushName(FACE);	// Push the category name in the stack
	glPushName(-1);		// Face id in the category

	// Material
	setupColoredMaterial(Vector3f(0.95f, 0.95f, 0.95f));

	// Triangles
	for (int i = 0; i < face.cols(); i++)
	{
		// Picking
		glLoadName(i);	// Replace the name for the i-the face

		// Material for selected objects
		if (!faceWithGapMesh && Df[i] != -1)	setupColoredMaterial(selectionColor[Df[i]]);

		glBegin(GL_TRIANGLES);
		for (int j = 0; j < 3; j++)
		{
			glNormal3fv(vertexNormal.col(face(j, i)).data());
			glVertex3fv(vertex.col(face(j, i)).data());
		}
		glEnd();

		// Material for non-selected objects
		if (!faceWithGapMesh && Df[i] != -1)
			setupColoredMaterial(Vector3f(0.95f, 0.95f, 0.95f));
	}

	// Picking
	glPopName();
	glPopName();
}

void
drawFaceWithGapMesh()
{
	// Material
	setupColoredMaterial(Vector3f(0, 1, 1));

	// Triangles
	for (int i = 0; i < face.cols(); i++)
	{
		// Material for selected objectes
		if (Df[i] != -1)
		{
			glDisable(GL_LIGHTING);
			glColor3fv(selectionColor[Df[i]].data());
		}
		else     continue;

		glBegin(GL_TRIANGLES);
		glNormal3fv(faceNormal.col(i).data());	// Face normal
		for (int j = 0; j < 3; j++)
			glVertex3fv(faceVertex.col(3*i+j).data());
		glEnd();

		// Material for non-selected objects
		if (Df[i] != -1)
		{
			glEnable(GL_LIGHTING);
			setupColoredMaterial(Vector3f(0, 1, 1));
		}
	}
}

void
drawVertices()
{
	// Picking
	glPushName(VERTEX);	// Push the category name in the stack
	glPushName(-1);		// Face id in the category

	// Material
	glDisable(GL_LIGHTING);
	glColor3f(0.2f, 0.2f, 0.2f);

	// Point size
	glPointSize(7 * dpiScaling);

	// Edges (i, j): i < j
	for (int i = 0; i < vertex.cols(); i++)
	{
		// Picking
		glLoadName(i);	// Replace the name for the i-the vertex

		// Material for selected objects
		if (Dv[i] != -1)	glColor3fv(selectionColor[Dv[i]].data());

		glBegin(GL_POINTS);
		glVertex3fv(vertex.col(i).data());	// Start vertex
		glEnd();

		// Material for non-selected objects
		if (Dv[i] != -1)	glColor3f(0.2f, 0.2f, 0.2f);
	}

	// Material
	glEnable(GL_LIGHTING);

	// Picking
	glPopName();
	glPopName();
}


void
drawEdges()
{
	// Picking
	glPushName(EDGE);	// Push the category name in the stack
	glPushName(-1);		// Edge id in the category

	// Material
	glDisable(GL_LIGHTING);
	glColor3f(0, 0, 0);

	// Point size
	glLineWidth(1.5f * dpiScaling);

	// Edges
	for (int e = 0; e < nEdges; e++)
	{
		int i = E[e].first;
		int j = E[e].second;

		// Picking
		glLoadName(e);	// Replace the name for the n-the edge

		// Material for selected objects
		if (De[e] != -1)
		{
			glLineWidth(2.0f * 1.5f * dpiScaling);
			assert(De[e] < 6);
			glColor3fv(selectionColor[De[e]].data());
		}

		glBegin(GL_LINES);
		glNormal3fv(vertexNormal.col(i).data());
		glVertex3fv(vertex.col(i).data());	// Start vertex

		glNormal3fv(vertexNormal.col(j).data());
		glVertex3fv(vertex.col(j).data());	// Start vertex
		glEnd();

		// Material for non-selected objects
		if (De[e] != -1)
		{
			glLineWidth(1.5f * dpiScaling);
			glColor3f(0, 0, 0);
		}
	}

	// Material
	glEnable(GL_LIGHTING);

	// Picking
	glPopName();
	glPopName();
}

void
render(GLFWwindow* window, bool selectionMode)
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

	// Picking
	if (selectionMode)
	{
		glInitNames();	// Initialize the same stack
	}

	// Backgroud color
	glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2], up[0], up[1], up[2]);

	// Axes
	if (!selectionMode && axes)
	{
		glDisable(GL_LIGHTING);
		drawAxes(AXIS_LENGTH, AXIS_LINE_WIDTH * dpiScaling);
	}

	// Smooth shading
	glShadeModel(GL_SMOOTH);

	// Lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	setupLight(light);

	// Rotation about the y-axis
	glRotatef(theta / float(M_PI) * 180.0f, 0, 1, 0);	// Radian to degree

	// Polygon offset
	glEnable(GL_POLYGON_OFFSET_FILL);

	// Draw the mesh after setting up the material
	if (!selectionMode && faceWithGapMesh)
	{
		glPolygonOffset(1.0f, 1.0f);
		drawFaceWithGapMesh();
	}

	// Draw all the faces
	glPolygonOffset(2.0f, 1.0f);
	drawFaces();

	// Draw all the edges once
	drawEdges();

	// Draw all the vertices
	drawVertices();

	if (inputMode == InputMode::DRAGGING)
		drawRectangle();
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
setupLight(const Vector4f& p)
{
	GLfloat ambient[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
	GLfloat diffuse[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat specular[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0, GL_POSITION, p.data());
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

			// Face with gap mesh/mesh
		case GLFW_KEY_G:		faceWithGapMesh = !faceWithGapMesh; break;

			// Gap increase/decrease
		case GLFW_KEY_UP:
			gap = min(gap + 0.05f, 0.5f);
			buildShrunkenFaces(vertex, faceVertex);
			break;

		case GLFW_KEY_DOWN:
			gap = max(gap - 0.05f, 0.05f);
			buildShrunkenFaces(vertex, faceVertex);
			break;

			// n-ring
		case GLFW_KEY_0:	nRing = 0; break;
		case GLFW_KEY_1:	nRing = 1; break;
		case GLFW_KEY_2:	nRing = 2; break;
		case GLFW_KEY_3:	nRing = 3; break;
		case GLFW_KEY_4:	nRing = 4; break;
		case GLFW_KEY_5:	nRing = 5; break;

			// Component selection
		case GLFW_KEY_V:	pickMode = VERTEX;	break;
		case GLFW_KEY_E:	pickMode = EDGE;	break;
		case GLFW_KEY_F:	pickMode = FACE;	break;

			// Antialiasing on.off
		case GLFW_KEY_A:	vertexAdjacent = !vertexAdjacent; break;

			// Normal vector
		case GLFW_KEY_N:	useFaceNormal = !useFaceNormal; break;

			// Back face culling
		case GLFW_KEY_B:	bfcEnabled = !bfcEnabled; break;

			// Axes on/off
		case GLFW_KEY_X:	axes = !axes;	break;

			// Diagnosis on/off
		case GLFW_KEY_D:	diagnosis = !diagnosis;	break;
		}
	}
}

int
findNearestHits(int hits, GLuint selectBuffer[])
{
	if (diagnosis)	cout << "hits = " << hits << endl;

	int		name = -1;
	float	nearest = 2.0;	// z1, z2 in [0, 1]

	int		index = 0;
	for (int i = 0; i < hits; i++)
	{
		int		n = selectBuffer[index + 0];						// # of names
		float	z1 = (float)selectBuffer[index + 1] / 0xffffffff;	// min depth value
		float	z2 = (float)selectBuffer[index + 2] / 0xffffffff;	// max depth value

		if (n >= 2)	// Category and component
		{
			int		categoryName = selectBuffer[index + 3];			// Category
			if (categoryName == pickMode)
			{
				int		componentName = selectBuffer[index + 4];			// Component

				switch (pickMode)
				{
				case VERTEX:	Dv[componentName] = 0; pickedNeighborVertices.insert({ componentName, 0 }); break;
				case EDGE:		De[componentName] = 0; pickedNeighborEdges.insert({ componentName, 0 }); break;
				case FACE:		Df[componentName] = 0; pickedNeighborFaces.insert({ componentName, 0 }); break;
				case NONE:		break;
				}

				// Choose the nearest one only
				if (z1 < nearest) { nearest = z1; name = componentName; }

				if (diagnosis)
				{
					cout << "\t# of names = " << n << endl;
					cout << "\tz1 = " << z1 << endl;
					cout << "\tz2 = " << z2 << endl;
					cout << "\tindex: " << index << endl;
					cout << "\tnames: ";
					for (int j = 0; j < n; j++)
						cout << selectBuffer[index + 3 + j] << " ";
					cout << endl;
				}
			}
		}

		// To the next available one
		index += (3 + n);
	}
	if (diagnosis)	cout << "picked = " << name << endl;

	return	name;
}

int
selectObjects(GLFWwindow* window, double x, double y, double regionW, double regionH)
{
	// Width and height of picking region in window coordinates
	double	delX;
	double	delY;

	// Width and height of picking region in window coordinates
	if (regionW > 0)		delX = regionW;
	else if (regionW == 0)		delX = 1;
	else						delX = -regionW;
	if (regionH > 0)		delY = regionH;
	else if (regionH == 0)		delY = 1;
	else						delY = -regionH;

	// Maximum 64 selections: 5 int for one selection
	GLuint	selectBuffer[256 * 256];
	glSelectBuffer(256 * 256, selectBuffer);

	// Current viewport
	GLint	viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	// Backup the projection matrix
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();

	glLoadIdentity();

	// Creates a projection matrix that can be used to restrict drawing to
	// a small region of the viewport
	gluPickMatrix(x, viewport[3] - y, delX, delY, viewport);	// y screen to viewport

	// Exploit the projection matrix for normal rendering
	setupProjectionMatrix();

	// Enter selection mode
	glRenderMode(GL_SELECT);

	// Render the objects for selection
	render(window, true);

	// Return to normal rendering mode and getting the picked object
	GLint	hits = glRenderMode(GL_RENDER);
	int	name = findNearestHits(hits, selectBuffer);

	//  Restore the projection matrix
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	return name;
}


void
select(GLFWwindow* window, double x, double y, double regionW, double regionH)
{
	// Reset the previous selections
	if (picked != -1)
	{
		for (auto& f : pickedNeighborFaces)
		{
			assert(f.first < face.cols());
			Df[f.first] = -1;
		}

		for (auto& e : pickedNeighborEdges)
		{
			assert(e.first < nEdges);
			De[e.first] = -1;
		}

		for (auto& v : pickedNeighborVertices)
		{
			assert(v.first < vertex.cols());
			Dv[v.first] = -1;
		}
	}

	// Retrieve the selected object
	picked = selectObjects(window, x, y, regionW, regionH);
}

void
mouseButton(GLFWwindow* window, int button, int action, int mods)
{
	// Mouse cursor position in the screen coordinate system
	double	xs, ys;
	glfwGetCursorPos(window, &xs, &ys);

	if (action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_LEFT)
	{
		inputMode = InputMode::DRAGGING;	// Dragging starts

		point[0][0] = xs;	point[0][1] = ys;	// Start point
		point[1][0] = xs;	point[1][1] = ys;	// End point
	}

	// The left button of the mouse is released.
	if (action == GLFW_RELEASE && button == GLFW_MOUSE_BUTTON_LEFT)
	{
		inputMode = InputMode::COMPLETE;	// Dragging ends

		point[1][0] = xs;		point[1][1] = ys;	// End point

		// Mouse cursor position in the framebuffer coordinate
		double x0 = point[0][0] * dpiScaling;	//	for 
		double y0 = point[0][1] * dpiScaling;	//	for HiDPI
		double x1 = point[1][0] * dpiScaling;	//	for 
		double y1 = point[1][1] * dpiScaling;	//	for HiDPI

		// Retrieve the selected object
		select(window, (x0 + x1) / 2, (y0 + y1) / 2, x0 - x1, y0 - y1);
	}
}

void
mouseMove(GLFWwindow* window, double xs, double ys)
{
	// Update the end point while dragging
	if (inputMode == InputMode::DRAGGING)
	{
		point[1][0] = xs;	
		point[1][1] = ys;
	}
}
