
#include "glSetup.h"

#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>
using namespace std;

#ifdef _WIN32
#define _USE_MATH_DEFINES   // To include the definition of M_PI in math.h
#endif

#include <math.h>

void init();
void quit();
bool solveIK(const Vector3f& ee_goal);
void render(GLFWwindow* window);
void keyboard(GLFWwindow* window, int key, int code, int action, int mods);
void mouse(GLFWwindow* window, int button, int action, int mods);

// Light configuration
Vector4f    light(0.0, 0.0, 5.0, 1.0);     // Light position

// Global coordinate frame
float   AXIS_LENGTH = 3;
float   AXIS_LINE_WIDTH = 2;

// Colors
GLfloat bgColor[4] = { 1, 1, 1, 1 };

// Sphere
GLUquadricObj* sphere = NULL;
GLUquadricObj* cylinder = NULL;

// joints and links
const int	nLinks = 3;
float linkLength[nLinks] = { 0.4f, 0.4f, 0.4f };
float jointAngle[nLinks] = { 0.2f, 0.2f, 0.4f }; // Radian

const int	nlimbs = 2;
float limblinkLength[nLinks] = { 0.4f, 0.4f };
float limbjointAngle[nLinks] = { -0.6f, -0.4f }; // Radian

// Global transformation of the articulated figure
Vector3f    basePosition(0.0f, -0.7f, 0.0f);

// Forward Kinematics control joint
int selectedJoint = 0;

// Inverse kinematics
bool        target = false;
bool        target2 = false;
int         numIterations = 0;
int         numIterations2 = 0;
bool        outofreach = false;
Vector3f    ee_goal;
Vector3f    ee_goal2;

// Constants for IK
const float THRESHOLD_RATIO = 1.0E-4f;              // Accuracy threshold ratio
const float MAX_ANGLE_CHANGE = float(M_PI) / 4;     // Max dtheta per IK iteration
const float MAX_DX_RATIO = 0.5f;                    // Max dx ratio wrt a link for clamping
const float lambda = 0.2f;                          // Lambda for the damped least squares method

enum SolutionMethod {
    JACOBIAN_TRANSPOSE = 0,                         // Most simple
    PSEUDOINVERSE,                                  // Most efficient in the reachable workspace
    DAMPED_LEAST_SQUARES,                           // Most robust
}; SolutionMethod method = DAMPED_LEAST_SQUARES;

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
   glfwSetMouseButtonCallback(window, mouse);

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
      if (target)
      {
         // Solve the inverse kinematics problem
         bool solved = solveIK(ee_goal);   numIterations++; numIterations2++;

         // If solved, then turn off the target pointer in the screen
         if (solved) { target = false; target2 = false; }
      }

      render(window);          // Draw one frames  
      glfwSwapBuffers(window); // Swap buffers
      glfwPollEvents();       // Events
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

   cylinder = gluNewQuadric();
   gluQuadricDrawStyle(cylinder, GLU_FILL);
   gluQuadricNormals(cylinder, GLU_SMOOTH);
   gluQuadricOrientation(cylinder, GLU_OUTSIDE);
   gluQuadricTexture(cylinder, GL_FALSE);

   // Keyboard and mouse
   cout << "Keyboard Input: [1:3] for joint selection" << endl;
   cout << "Keyboard Input: left/right for adjusting joint angles" << endl;
   cout << "Keyboard Input: j for the jacobian transpose method" << endl;
   cout << "Keyboard Input: p for the pseudoinverse method" << endl;
   cout << "Keyboard Input: d for the damped least squares method" << endl;
   cout << "Keyboard Input: o for Determining out-of-reach state on/off" << endl;
   cout << "Mouse Input: desired position of the end-effector" << endl;
}

void
quit()
{
   // Delete quadric shapes
   gluDeleteQuadric(sphere);
   gluDeleteQuadric(cylinder);
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

// Material
void 
setupMaterial()
{
   //Material
   GLfloat mat_ambient[4] = { 0.1f, 0.1f, 0.1f, 1};
   GLfloat mat_specular[4] = { 0.5f, 0.5f, 0.5f,1 };
   GLfloat mat_shininess = 128.0f;

   glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
   glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
   glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
}

void
setDiffuseColor(const Vector3f& color)
{
   GLfloat mat_diffuse[4] = { color[0], color[1], color[2],1 };
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

// Draw a cyliner after setting up its material
void
drawCylinder(float radius, float height, const Vector3f& color)
{
   // Material
   setDiffuseColor(color);

   // Sphere using GLU quadrics
   gluCylinder(cylinder, radius, radius, height, 72, 5);
}

void
drawJoint(float radius, const Vector3f& color)
{
   glPushMatrix();

   // Front sphere
   glTranslatef(0, 0, radius);       // Translation using OpenGL glTranslatef();
   drawSphere(radius, color);

   // Rear sphere
   glTranslatef(0, 0, -2.0f*radius); // -2.0 to avoid calling push/pop matrix
   drawSphere(radius, color);

   glPopMatrix();

   // Cylinder
   drawCylinder(radius, radius, color);
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
drawCube(float width, float height, float depth, const Vector3f& color)
{
    // Material 
    setDiffuseColor(color);

    // Cube
    glPushMatrix();
    glScalef(width, height, depth);

    glBegin(GL_QUADS);

    // Front
    glNormal3f(0, 0, 1);

    glVertex3f(-0.5f, -0.5f,  0.5f);
    glVertex3f( 0.5f, -0.5f,  0.5f);
    glVertex3f( 0.5f,  0.5f,  0.5f);
    glVertex3f(-0.5f,  0.5f,  0.5f);

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

    glVertex3f(-0.5f,  0.5f,  0.5f);
    glVertex3f( 0.5f,  0.5f,  0.5f);
    glVertex3f (0.5f,  0.5f, -0.5f);
    glVertex3f(-0.5f,  0.5f, -0.5f);

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

bool
solveIK(const Vector3f& EE_goal)
{
   // The joint axes are all the same in this example.
   Vector3f jointAxis(0, 0, 1);

   // Joint Positions
   Vector3f jointPosition[nLinks];
   Vector3f limbjointPosition[nlimbs];

   // Transform
   Matrix4f T;
   Matrix4f H;

   // Base transform
   T.setIdentity();
   T.block<3, 1>(0, 3) = basePosition;      // 3x1 translation vector
   
   H.setIdentity();

   // Joints and links
   Matrix4f     T_joint, T_link;
   Matrix4f     H_joint, H_link;
   for (int i = 0; i < nLinks; i++)
   {
      // the position of the joint: P_i
      jointPosition[i] = (T * Vector4f(0, 0, 0, 1)).block<3, 1>(0, 0);

      // joint transform using an Eigen 4x4 matrix. 3x3 rotation matrix
      T_joint.setIdentity();
      T_joint.block<3, 3>(0, 0) = Matrix3f(AngleAxisf(jointAngle[i], Vector3f::UnitZ()));
      
      if (i == 1)
      {
          H = T;
          for (int j = 0; j < nlimbs; j++)
          {
              limbjointPosition[j] = (H * Vector4f(0, 0, 0, 1)).block<3, 1>(0, 0);
              H_joint.setIdentity();
              H_joint.block<3, 3>(0, 0) = Matrix3f(AngleAxisf(limbjointAngle[j], Vector3f::UnitZ()));
              H = H * H_joint;

              H_link.setIdentity();
              H_link.block<3, 1>(0, 3) = Vector3f(0.0, limblinkLength[j], 0.0f);
          
              H = H * H_link;
          }
      }
      T = T * T_joint;
   
      // Link transform using an Eigen 4x4 matrix. 3x1 translation Vector
      T_link.setIdentity();
      T_link.block<3, 1>(0, 3) = Vector3f(0.0, linkLength[i], 0.0);
      T = T * T_link;
   }

   // the current position of the end-effector
   // bVector3f e_current = T.block<3, 3>(0, 0) * Vector3f(0, 0, 0) + T.block<3, 1>(0, 3);
   Vector3f ee_current = (T * Vector4f(0, 0, 0, 1)).block<3, 1>(0, 0);
   Vector3f ee_current2 = (H * Vector4f(0, 0, 0, 1)).block<3, 1>(0, 0);


   // Jacobian
   MatrixXf     J(3, nLinks);
   for (int i = 0; i < nLinks; i++)     // Iterate all the joints
   {
      // The i-th column of the Jacobian matrix
      J.block<3, 1>(0, i) = jointAxis.cross(ee_current - jointPosition[i]);
   }

   MatrixXf     J2(3, nlimbs);
   for (int i = 0; i < nlimbs; i++)     // Iterate all the joints
   {
       // The i-th column of the Jacobian matrix
       J2.block<3, 1>(0, i) = jointAxis.cross(ee_current2 - limbjointPosition[i]);
   }

   // Delta x
   Vector3f     dx = ee_goal - ee_current;
   Vector3f     dx2 = ee_goal2 - ee_current2;

   // Clamping dx so that it is less than MAZ_DX_RATIO * linkLength[0]
   float    l = dx.norm();
   if (l > linkLength[0] * MAX_DX_RATIO)    dx *= (linkLength[0] * MAX_DX_RATIO) / l;

   float    l2 = dx2.norm();
   if (l2 > limblinkLength[0] * MAX_DX_RATIO)    dx2 *= (limblinkLength[0] * MAX_DX_RATIO) / l2;


   // Check to see if the solution is already found.
   if (dx.norm() < THRESHOLD_RATIO * linkLength[0] && dx2.norm() < THRESHOLD_RATIO * limblinkLength[0] && target2)     // Relative error wrt a link
   {
       //cout << "IK iterations = " << numIterations2 - numIterations << endl;
       return true;
   }

   // Solve IK
   VectorXf dtheta;
   float    dsum = 0.0;
   VectorXf dtheta2;
   float    dsum2 = 0.0;

   switch (method)
   {
   case JACOBIAN_TRANSPOSE:      // the Jacobuan transpose method
      {
         VectorXf Jtdx = J.transpose() * dx;
         VectorXf JJtdx = J * Jtdx;
         float alpha = dx.dot(JJtdx) / JJtdx.dot(JJtdx);
         dtheta = alpha * Jtdx;

         VectorXf Jtdx2 = J2.transpose() * dx2;
         VectorXf JJtdx2 = J2 * Jtdx2;
         float alpha2 = dx2.dot(JJtdx2) / JJtdx2.dot(JJtdx2);
         dtheta = alpha2 * Jtdx2;
      } 
      break;

   case PSEUDOINVERSE:            // The pseudoinverse method
      {
         dtheta = J.bdcSvd(ComputeThinU | ComputeThinV).solve(dx);
         dtheta2 = J2.bdcSvd(ComputeThinU | ComputeThinV).solve(dx2);
      } 
      break;

   case DAMPED_LEAST_SQUARES:      // The damped least squares method
      {
         int m = int(J.rows());
         MatrixXf A = (J * J.transpose() + lambda * lambda * MatrixXf::Identity(m, m));
         VectorXf f = A.colPivHouseholderQr().solve(dx);
         dtheta = J.transpose() * f;

         int m2 = int(J2.rows());
         MatrixXf A2 = (J2 * J2.transpose() + lambda * lambda * MatrixXf::Identity(m2, m2));
         VectorXf f2 = A2.colPivHouseholderQr().solve(dx2);
         dtheta2 = J2.transpose() * f2;
      }
      break;
   }

   // Update the joint angles
   for (int j = 0; j < nLinks; j++)
   {
      // Clmaping dtheta[j] so that it becomes less than MAX_ANGLE_CHANGE
      float l = fabs(dtheta[j]);
      if (l > MAX_ANGLE_CHANGE)   dtheta[j] *= MAX_ANGLE_CHANGE / l;

      // Joint angle angle within [-M_PI, M_PI]
      jointAngle[j] += dtheta[j];
      dsum += l;
      if (jointAngle[j] >  M_PI) jointAngle[j] -= 2 * float(M_PI);
      if (jointAngle[j] < -M_PI) jointAngle[j] += 2 * float(M_PI);
   }

   for (int j = 0; j < nlimbs; j++)
   {
       // Clmaping dtheta[j] so that it becomes less than MAX_ANGLE_CHANGE
       float l = fabs(dtheta2[j]);
       if (l > MAX_ANGLE_CHANGE)   dtheta2[j] *= MAX_ANGLE_CHANGE / l;

       // Joint angle angle within [-M_PI, M_PI]
       limbjointAngle[j] += dtheta2[j];
       dsum2 += l;
       if (limbjointAngle[j] > M_PI) limbjointAngle[j] -= 2 * float(M_PI);
       if (limbjointAngle[j] < -M_PI) limbjointAngle[j] += 2 * float(M_PI);
   }

   // Exercise : Check to see if a solution is found when the ee_goal is out of reach.
   // Hint : You can consider the chages of the joint angles.
      // Check to see if the solution is already found.
   if (outofreach && dsum > 0.0 && dsum < THRESHOLD_RATIO * linkLength[0] && outofreach && dsum2 > 0.0 && dsum2 < THRESHOLD_RATIO * limblinkLength[0])     // Relative error wrt a link
   {
       //cout << "IK iterations = " << numIterations << endl;
       return true;
   }

   //if (outofreach && dsum2 > 0.0 && dsum2 < THRESHOLD_RATIO * limblinkLength[0])     // Relative error wrt a link
   //{
   //    cout << "IK iterations = " << numIterations2 - numIterations << endl;
   //    return true;
   //}

   return false;
}

void render(GLFWwindow* window)
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
   

   float radiusjoint = 0.05f;
   float radiusLink = 0.04f;

   // Desired Position of the end-effector
   if (target)
   {
      glTranslatef(ee_goal[0], ee_goal[1], ee_goal[2]);
      drawJoint(radiusjoint, Vector3f(0.95f, 0, 0.95f));
   }

   // Transform
   Matrix4f T;
   Matrix4f H;
   
   // Base transform
   T.setIdentity();
   T.block<3, 1>(0, 3) = basePosition;   // 3x1 translation Vector

   H.setIdentity();

   // Geometry of the base
   glLoadMatrixf(T.data());
   drawBase(radiusLink, Vector3f(0.0f, 0.5f, 0.5f));

   // Joints and links
   Matrix4f T_joint, T_link;
   Matrix4f H_joint, H_link;
   for (int i = 0; i < nLinks; i++) 
   {
      // Joint transform using an Eigen 4x4 matirx. 3x3 rotation matrix
      T_joint.setIdentity();
      T_joint.block<3, 3>(0, 0) = Matrix3f(AngleAxisf(jointAngle[i], Vector3f::UnitZ()));
      if (i == 1)   H = T;
      T = T * T_joint;

      // Geometry of the joint and link
      glLoadMatrixf(T.data());
      drawJoint(radiusjoint, Vector3f(0.95f, 0, 0));                                // Draw a Joint
      drawLink(radiusLink, linkLength[i], Vector3f(0, 0.95f, 0.95f));               // Draw a Link

      if (i == 1)
      {
          glPushMatrix();
          for (int j = 0; j < nlimbs; j++)
          {
              H_joint.setIdentity();
              H_joint.block<3, 3>(0, 0) = Matrix3f(AngleAxisf(limbjointAngle[j], Vector3f::UnitZ()));
              H = H * H_joint;

              glLoadMatrixf(H.data());
              drawJoint(radiusjoint, Vector3f(0.95f, 0, 0));                                // Draw a Joint
              drawLink(radiusLink, limblinkLength[j], Vector3f(0, 0.95f, 0.95f));               // Draw a Link

              H_link.setIdentity();
              H_link.block<3, 1>(0, 3) = Vector3f(0.0, limblinkLength[j], 0.0f);

              H = H * H_link;
          }
          glLoadMatrixf(H.data());
          drawJoint(radiusjoint, Vector3f(0.95f, 0.95f, 0.0f));    // Draw the EE as a joint

          glPopMatrix();
      }

      // Link transform using Eigen 4x4 matirx: 3x1 translation vector
      T_link.setIdentity();
      T_link.block<3, 1>(0, 3) = Vector3f(0.0, linkLength[i], 0.0f);
      T = T * T_link;
   }

   // End-effector
   glLoadMatrixf(T.data());
   drawJoint(radiusjoint, Vector3f(0.95f, 0.95f, 0.0f));    // Draw the EE as a joint

   if (!target2)
       ee_goal2 = (H * Vector4f(0, 0, 0, 1)).block<3, 1>(0, 0);
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

      // Forward kinematics control
      case GLFW_KEY_1:      selectedJoint = 0;  break;
      case GLFW_KEY_2:      selectedJoint = 1;  break;
      case GLFW_KEY_3:      selectedJoint = 2;   break;
      case GLFW_KEY_4:      selectedJoint = 3;   break;
      case GLFW_KEY_5:      selectedJoint = 4;   break;

      case GLFW_KEY_LEFT:       if(selectedJoint >= 3) limbjointAngle[selectedJoint-3] += 0.1f; else jointAngle[selectedJoint] += 0.1f;  break;
      case GLFW_KEY_RIGHT:      if (selectedJoint >= 3) limbjointAngle[selectedJoint - 3] -= 0.1f; else jointAngle[selectedJoint] -= 0.1f;  break;
   
      // Inverse Kinematics method
      case GLFW_KEY_J:      method = JACOBIAN_TRANSPOSE;    cout << "\nInverse Kinematics method is JACOBIAN_TRANSPOSE\n" << endl; break;
      case GLFW_KEY_P:      method = PSEUDOINVERSE;         cout << "\nInverse Kinematics method is PSEUDOINVERSE\n" << endl; break;
      case GLFW_KEY_D:      method = DAMPED_LEAST_SQUARES;  cout << "\nInverse Kinematics method is DAMPED_LEAST_SQUARES\n" << endl; break;
      case GLFW_KEY_O:      outofreach = !outofreach;  if (outofreach) cout << "\n""Out of reach"" state can be determined\n" << endl; else cout << "\n""Out of reach"" state can't be determined\n" << endl; break;
      }

      // Target off
      target = false;
      target2 = false;
   }
}

void
mouse(GLFWwindow* window, int button, int action, int mods)
{
   if (action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_LEFT)
   {
      // Target on
       if (!target && !target2)
       {
           target = true;
           numIterations = 0;

           // In the screen coordinate
           double xpos, ypos;
           glfwGetCursorPos(window, &xpos, &ypos);
           ee_goal = Vector3f(float(xpos), float(ypos), 0);
           //cout << "Mouse at (" << xpos << "," << ypos << ")" << endl;

           // In the workspce. see reshape() in glSetup.cpp
           float aspect = (float)screenW / screenH;
           ee_goal[0] = 2.0f * (ee_goal[0] / screenW - 0.5f) * aspect;
           ee_goal[1] = -2.0f * (ee_goal[1] / screenH - 0.5f);

           //cout << "EE : " << ee_goal.transpose() << endl;
       }

       else if (target && !target2)
       {
           target2 = true;
           numIterations = 0;
           numIterations2 = 0;

           // In the screen coordinate
           double xpos, ypos;
           glfwGetCursorPos(window, &xpos, &ypos);
           ee_goal2 = Vector3f(float(xpos), float(ypos), 0);
           //cout << "Mouse at (" << xpos << "," << ypos << ")" << endl;

           // In the workspce. see reshape() in glSetup.cpp
           float aspect = (float)screenW / screenH;
           ee_goal2[0] = 2.0f * (ee_goal2[0] / screenW - 0.5f) * aspect;
           ee_goal2[1] = -2.0f * (ee_goal2[1] / screenH - 0.5f);
           //cout << "EE : " << ee_goal.transpose() << endl;
       }
   }
}

