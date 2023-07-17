
#include "glSetup.h"

#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>
using namespace std;

#ifdef _WIN32
#define _USE_MATH_DEFINES	// To include the definition of M_PI in math.h
#endif

#include <math.h>

void init();
void quit();
void keyboard(GLFWwindow* window, int key, int code, int action, int mods);
void render(GLFWwindow* window);

// Light configuration
Vector4f light(0.0f, 0.0f, 5.0f, 1.0f); // Light position

// Global coordinate frame
float AXIS_LENGTH = 3;
float AXIS_LINE_WIDTH = 2;

// Colors
GLfloat bgColor[4] = { 1,1,1,1 };

// Sphere
GLUquadricObj* sphere = NULL;
GLUquadricObj* cylinder = NULL;

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

	// Detph test
	glEnable(GL_DEPTH_TEST);

	// Normal vectors are normalized after transformation
	glEnable(GL_NORMALIZE);

	// Back face culling
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);

	// Viewport and perspective setting
	reshape(window, windowW, windowH);

	// Intialization - Main loop - Finalization
	init();
	
	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		render(window);          // Draw one frames  
		glfwSwapBuffers(window); // Swap buffers
		glfwPollEvents();		 // Events
	}

	//Finalization
	quit();
	
	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
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

	// Prepare quadric shapes
	cylinder = gluNewQuadric();
	gluQuadricDrawStyle(cylinder, GLU_FILL);
	gluQuadricNormals(cylinder, GLU_SMOOTH);
	gluQuadricOrientation(cylinder, GLU_OUTSIDE);
	gluQuadricTexture(cylinder, GL_FALSE);

	// Keyboard and mouse
	cout << "Keyboard Input: [1:3] for joint selection" << endl;
	cout << "Keyboard Input: left/right for adjusting joint angles" << endl;
	cout << "Keyboard Input: d for damping on/off" << endl;
}


void quit()
{
	// Delete quadric shapes
	gluDeleteQuadric(sphere);
	gluDeleteQuadric(cylinder);
}

// Light
void setupLight()
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

// Material
void
setupMaterial()
{
	//Material
	GLfloat mat_ambient[4] = { 0.1f, 0.1f, 0.1f, 1};
	GLfloat mat_specular[4] = { 0.5f, 0.5f, 0.5f, 1 };
	GLfloat mat_shininess = 128.0f;

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

// Draw a shpere after setting up its material
void
drawSphere(float radius, const Vector3f& color)
{
	// Material
	setDiffuseColor(color);

	// Sphere using GLU quadrics
	gluSphere(sphere, radius, 72, 72);
}

// Draw a shpere after setting up its material
void
drawCylinder(float radius,float height, const Vector3f& color)
{
	// Material
	setDiffuseColor(color);

	// Sphere using GLU quadrics
	gluCylinder(sphere, radius, radius, height, 72, 5);
}

void
drawJoint(float radius, const Vector3f& color)
{
	glPushMatrix();

	// Front sphere
	glTranslatef(0,0,radius);	      // Translation using OpenGL glTranslatef();
	drawSphere(radius, color);

	// Rear sphere
	glTranslatef(0, 0, -2.0f*radius); // -2.0 to avoid calling push/pop matrix
	drawSphere(radius, color);

	glPopMatrix();

	// Cylinder
	drawCylinder(radius, radius,color);
}

void
drawLink(float radius, float height, const Vector3f& color)
{
	glPushMatrix();

	// Rotation using OpenGL glRotatef(angle_in_degree, axis_x, axis_y, axis_z)
	glRotatef(-90, 1, 0, 0);

	// Draw sphere using gluCylinder() after setting up the material
	drawCylinder(radius, height, color);

	glPopMatrix();
}

void
drawCube(float width, float height,float depth, const Vector3f& color)
{
	// Material 
	setDiffuseColor(color);

	// Cube
	glPushMatrix();
	glScalef(width, height, depth);
	
	glBegin(GL_QUADS);

	// Front
	glNormal3f(0, 0, 1);

	glVertex3f(-0.5f, -0.5f, 0.5f);
	glVertex3f( 0.5f, -0.5f, 0.5f);
	glVertex3f( 0.5f,  0.5f, 0.5f);
	glVertex3f(-0.5f,  0.5f, 0.5f);

	// Back
	glNormal3f(0, 0, -1);

	glVertex3f(-0.5f,  0.5f, -0.5f);
	glVertex3f( 0.5f,  0.5f, -0.5f);
	glVertex3f( 0.5f, -0.5f, -0.5f);
	glVertex3f(-0.5f, -0.5f, -0.5f);

	// Right
	glNormal3f(1, 0, 0);

	glVertex3f(0.5f, -0.5f,  0.5f);
	glVertex3f(0.5f, -0.5f, -0.5f);
	glVertex3f(0.5f,  0.5f, -0.5f);
	glVertex3f(0.5f,  0.5f,  0.5f);

	// Left
	glNormal3f(-1, 0, 0);

	glVertex3f(-0.5f,  0.5f,  0.5f);
	glVertex3f(-0.5f,  0.5f, -0.5f);
	glVertex3f(-0.5f, -0.5f, -0.5f);
	glVertex3f(-0.5f, -0.5f,  0.5f);

	// Top
	glNormal3f(0, 1, 0);

	glVertex3f(-0.5f, 0.5f,  0.5f);
	glVertex3f( 0.5f, 0.5f,  0.5f);
	glVertex3f( 0.5f, 0.5f, -0.5f);
	glVertex3f(-0.5f, 0.5f, -0.5f);

	// Bottom
	glNormal3f(0, -1, 0);

	glVertex3f(-0.5f, -0.5f, -0.5f);
	glVertex3f( 0.5f, -0.5f, -0.5f);
	glVertex3f( 0.5f, -0.5f,  0.5f);
	glVertex3f(-0.5f, -0.5f,  0.5f);

	glEnd();

	glPopMatrix();
}

void
drawBase(float radiusLink, const Vector3f& color)
{
	float baseLinkHeight = 0.15f;
	float baseWidth= 1.0f;
	float baseHeight = 0.1f;
	float baseDepth= 1.0f;

	glPushMatrix();
	glTranslatef(0, -(baseLinkHeight + baseHeight / 2), 0);
	drawLink(radiusLink, (baseLinkHeight + baseHeight / 2), color);
	drawCube(baseWidth, baseHeight, baseDepth, color);
	glPopMatrix();
}

// Joint and links
const int	nLinks = 3;
float linkLength[nLinks] = { 0.4f, 0.4f, 0.4f };
float jointAngle[nLinks] = { -0.1f,-0.1f,-0.1f }; // Radian

// Global transformation of the articulated figure
Vector3f	basePosition(0.0f, -0.7f, 0.0f);

// Forward kinematics control joint
int selectedJoint = 0;

float radiusJoint = 0.05f;
float radiusLink = 0.04f;

void drawKinematicModelUsingOpenGL();
void drawKinematicModelUsingEigen();

float	radian2degree(float r) { return float(180.0 * r / M_PI); }

// Drawing method
bool useOpenGL = true; // OpenGL/Eigen

void 
render(GLFWwindow* window)
{
	//Background color
	glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// ModelView matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Lighting
	setupLight();

	// Material
	setupMaterial();
	
	// Draw
	if (useOpenGL) drawKinematicModelUsingOpenGL();
	else           drawKinematicModelUsingEigen();
}

void 
drawKinematicModelUsingOpenGL()
{
	// Base transform using OpenGL glTranslatef()
	glTranslatef(basePosition.x(), basePosition.y(), basePosition.z());

	// Geometry of the base
	drawBase(radiusLink, Vector3f(0.0f, 0.5f, 0.5f));

	// Joints and links
	for (int i = 0; i < nLinks; i++)
	{
		// Joint transform
		glRotatef(radian2degree(jointAngle[i]), 0, 0, 1);

		// Geometry of the joint and link
		drawJoint(radiusJoint, Vector3f(0.95f, 0, 0));				    // Draw a joint
		drawLink(radiusLink, linkLength[i], Vector3f(0, 0.95f, 0.95f)); // Draw a link

		// Link transform
		glTranslatef(0, linkLength[i], 0);
	}

	// End - effector
	drawJoint(radiusJoint, Vector3f(0.95f, 0.95f, 0.0f)); // Draw the EE as joint
}

void
drawKinematicModelUsingEigen()
{
	// Base transform
	Matrix4f T;
	T.setIdentity();
	T.block<3, 1>(0, 3) = basePosition; // 3 x 1 translation vector

	// Geometry of the bases
	glLoadMatrixf(T.data());
	drawBase(radiusLink, Vector3f(0.0f, 0.5f, 0.5f));

	// Joint and links
	Matrix4f	T_joint, T_link;
	for (int i = 0; i < nLinks; i++)
	{
		// Joint transform using an Eigen 4x4 matrix. 3x3 rotation matrx
		T_joint.setIdentity();
		T_joint.block<3, 3>(0, 0) = Matrix3f(AngleAxisf(jointAngle[i], Vector3f::UnitZ()));
		T = T * T_joint;

		// Geometry of the joint and link
		glLoadMatrixf(T.data());
		drawJoint(radiusJoint, Vector3f(0.95f, 0, 0));	                //Draw a joint
		drawLink(radiusLink, linkLength[i], Vector3f(0, 0.95f, 0.95f)); // Draw a link
	
		// Link transform using an Eigen 4x4 matrix : 3x1 translation vector
		T_link.setIdentity();
		T_link.block<3, 1>(0, 3) = Vector3f(0.0, linkLength[i], 0.0);
		T = T * T_link;
	}

	// End - effector
	glLoadMatrixf(T.data());
	drawJoint(radiusJoint, Vector3f(0.95f, 0.95f, 0.0f)); // Draw the EE as a joint
}

Vector3f
computeEE()
{
	// Base transform()
	Matrix4f T;
	T.setIdentity();
	T.block<3, 1>(0, 3) = basePosition;	// 3x1 translation vector

	// Joints and links
	Matrix4f T_joint, T_link;
	for (int i = 0; i < nLinks; i++)
	{
		// Joint transform using an Eigen 4x4 matrix, 3x3 rotation matrix
		T_joint.setIdentity();
		T_joint.block<3, 3>(0, 0) = Matrix3f(AngleAxisf(jointAngle[i], Vector3f::UnitZ()));
		T = T * T_joint;

		// Link transform using an Eigen 4x4 matrix : 3x1 translation vector
		T_link.setIdentity();
		T_link.block<3, 1>(0, 3) = Vector3f(0.0, linkLength[i], 0.0);
		T = T * T_link;
	}

	// The current position of the end - effector
	Vector4f ee = T * Vector4f(0, 0, 0, 1);

	return ee.block<3, 1>(0, 0);
}


void 
keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT)
	{
		switch (key)
		{
			//Quit
		case GLFW_KEY_Q:
		case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GL_TRUE); break;

		// Joint
		case GLFW_KEY_1:  selectedJoint = 0;  break;
		case GLFW_KEY_2:  selectedJoint = 1;  break;
		case GLFW_KEY_3:  selectedJoint = 2;   break;

		//  Joint angles
		case GLFW_KEY_LEFT: jointAngle[selectedJoint] += 0.1f;  break;
		case GLFW_KEY_RIGHT:jointAngle[selectedJoint] -= 0.1f;  break;
	
		//Drawing method
		case GLFW_KEY_D: useOpenGL= !useOpenGL; break;
		}
	}
}


