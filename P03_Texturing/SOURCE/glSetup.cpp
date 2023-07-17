#include "glSetup.h"

#include <string.h>
#include <iostream>
using namespace std;

bool fullScreen = false;
bool noMenuBar = false;
bool perspectiveView = true;

float screenScale = 0.75f;      // Portion of the screen when not using full screen
int screenW = 0, screenH = 0;   // screenScale portion of the screen
int windowW, windowH;         // Framebuffer size
float aspect;               // Aspect ratio = windowW/windowH
float dpiScaling = 0;         // for HIDPI: windowW = dpiScaling x screenW

int vsync = 1;               // Vertical sync on/off

float fovy = 46.4f;            // fovy of 28mm lens in degree
//float fovy = 37.8f;         // fovy of 35mm lens in degree
//float fovy = 27.0f;         // fovy of 50mm lens in degree
//float fovy = 16.1f;         // fovy of 85mm lens in degree
//float fovy = 11.4f;         // fovy of 120mm lens in degree
//float fovy = 6.9f;         // fovy of 200mm lens in degree

float nearDist = 1.0f;
float farDist = 20.0f;

void errorCallback(int error, const char* description)
{
    cerr << "####" << description << endl;
}

void setupProjectionMatrix()
{
    if (perspectiveView) gluPerspective(fovy, aspect, nearDist, farDist);
    else glOrtho(-1.0 * aspect, 1.0 * aspect, -1.0, 1.0, -nearDist, farDist);
}

// w and h are width and height of the framebuffer
void reshape(GLFWwindow* window, int w, int h)
{
    aspect = (float)w / h;

    // Set the viewport
    windowW = w;
    windowH = h;
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Set up a projection matrix
    setupProjectionMatrix();

    // The Screen size is required for mouse interaction
    glfwGetWindowSize(window, &screenW, &screenH);
    cerr << "reshpae(" << w << ", " << h << ")";
    cerr << " with screen" << screenW << "x " << screenH << endl;
}

GLFWwindow* initializeOpenGL(int argc, char* argv[], GLfloat bgColor[4], bool modern)
{
    glfwSetErrorCallback(errorCallback); //오류 처리해주는 콜백

    // Init GLFW
    if (!glfwInit()) exit(EXIT_FAILURE); // 초기화 실패시 프로그램 종료.

    if (modern) // Enable OpenGL 4.1 in OS X
    {
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    }
    else // Enable OpenGl 2.1 in OS X
    {
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    }

    glfwWindowHint(GLFW_SAMPLES, 4);

    // Create the window ( OpenGl이 실행될 수 있는 Window )
    GLFWmonitor* monitor = glfwGetPrimaryMonitor(); // glfw는 여러 모니터도 지원.
    int monitorW, monitorH;
    glfwGetMonitorPhysicalSize(monitor, &monitorW, &monitorH);
    cerr << "Status: Monitor " << monitorW << "mm x" << monitorH << "mm" << endl; //모니터에 대한 정보 출력

    //// Full screen
    if (fullScreen) screenScale = 1.0;

    const GLFWvidmode* videoMode = glfwGetVideoMode(monitor);
    //인자를 안받으면 screenScale 이용해서 구한다.
    if (screenW == 0) screenW = int(videoMode->width * screenScale);
    if (screenH == 0) screenH = int(videoMode->height * screenScale);


    if (!fullScreen || !noMenuBar) monitor = NULL;
    GLFWwindow* window = glfwCreateWindow(screenW, screenH, argv[0], monitor, NULL);

    if (!window)
    {
        glfwTerminate();
        cerr << "Failed in glfwCreateWindow()" << endl;
        return NULL;
    }

    //Context
    glfwMakeContextCurrent(window);

    //Clear the background ASAP
    glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
    glClear(GL_COLOR_BUFFER_BIT);
    glFlush(); //당장 지워라
    glfwSwapBuffers(window); // 더블 버퍼링!! opengl에서는 프레임 버퍼를 두개 쓴다. 
    // 보고있는 화면과 보고있지 않은화면, 보고있지 않는 화면에 그려놓고 단번에 바꿔끼운다.
    //그려지는 과정을 보지 않으려고.

    //Check the size of the window
    glfwGetWindowSize(window, &screenW, &screenH);
    cerr << "Status: Screen " << screenW << "x " << screenH << endl;

    glfwGetFramebufferSize(window, &windowW, &windowH);
    cerr << "Status: Framebuffer " << windowW << "x " << windowH << endl;

    //DPI scaling
    if (dpiScaling == 0) dpiScaling = float(windowW) / screenW;

    //Callbacks 콜백이 무엇인지 구글링 하슈
    glfwSetFramebufferSizeCallback(window, reshape);

    //Get the OpenGl version and renderer
    cout << "Status: Renderer " << glGetString(GL_RENDERER) << endl;
    cout << "Status: Ventor " << glGetString(GL_VENDOR) << endl;
    cout << "Status: OpenGl" << glGetString(GL_VERSION) << endl;

    // Vertical sync...
    glfwSwapInterval(vsync); // 0 for immediate mode ( Tearing possible ) 화면 찢어지는 일 발생할 수도.
    //그게 싫다면 0이 아닌 숫자 스면된다.
    // 0이라면 가능한 빨리 화면을 리프레시 

    if (modern)
    {
        //GLSL version for shader loading
        cout << "Status: GLSL " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;

        //GLEW : Supported version can be verified in glew.sourceforge.net/basic.html
        cerr << "Status: GLEW " << glewGetString(GLEW_VERSION) << endl;

        // Initiallizing GLEW
        GLenum error = glewInit();
        if (error != GLEW_OK)
        {
            cerr << "ERROR: " << glewGetErrorString(error) << endl;
            return 0;
        }

    }
    return window;
}

void drawAxes(float l, float w)
{
    glLineWidth(w * dpiScaling);

    glBegin(GL_LINES);
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(1, 0, 0); //x-axis
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 1, 0); //y-axis
    glColor3f(1, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 1); //z-axis
    glEnd();
}