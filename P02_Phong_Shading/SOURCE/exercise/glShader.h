
#ifndef __GL_SHADER_H_
#define __GL_SHADER_H_

#include <GL/glew.h>				// OpenGL Extension Wrangler Libary
#include <GLFW/glfw3.h>

#include <Eigen/Dense>
using namespace Eigen;

bool	isOK(const char* message = NULL, const char* file = NULL, int line = -1,
			bool exitOnError = true, bool report = true);

// Create and delete the shaders and the program
void	createShaders(const char* vertexShaderFile, const char* fragmentShaderFile,
			GLuint& program, GLuint& vertexShader, GLuint& fragmentShader);
char*	readShader(const char* filename);
GLuint	createShaderFromFile(GLenum shaderType, const char* filename);
void	printShaderInfoLog(GLuint obj, const char* shaderFilename);
void	printProgramInfoLog(GLuint obj);
void	deleteShaders(GLuint program, GLuint vertexShader, GLuint fragmentShader);

// Get the location of a uniform parameter
int getUniformLocation(GLuint program, const char* name);
int getUniformLocation(GLuint program, const std::string& name);

// Set uniform parameters
int setUniformi(GLuint program, const std::string& name, int i);
int setUniform(GLuint program, const std::string& name, float f);
int setUniform(GLuint program, const std::string& name, const Vector2f& v);
int setUniform(GLuint program, const std::string& name, const Vector3f& v);
int setUniform(GLuint program, const std::string& name, const Vector4f& v);
int setUniform(GLuint program, const std::string& name, const Matrix3f& m);
int setUniform(GLuint program, const std::string& name, const Matrix4f& m);
int setUniformMatrix3fv(GLuint program, const char* name, const float* value);
int setUniformMatrix4fv(GLuint program, const char* name, const float* value);

void	createVBO(GLuint& vao, GLuint& indexId, GLuint& vertexId, GLuint& normalId);
void	createVBO(GLuint& vao, GLuint& indexId, GLuint& vertexId, GLuint& normalId,
				  GLuint& coordId);
int		uploadMesh2VBO(ArrayXXi& face, MatrixXf& vertex, MatrixXf& normal,
						GLuint vao, GLuint indexId, GLuint vertexId, GLuint normalId);
int		uploadMesh2VBO(ArrayXXi& face, MatrixXf& vertex, MatrixXf& normal,
						MatrixXf& texture, GLuint vao, GLuint indexId, GLuint vertexId,
						GLuint normalId, GLuint texId);
void	drawVBO(GLuint vao, int numTriangles);
void	deleteVBO(GLuint& vao, GLuint& indexId, GLuint& vertexId, GLuint& normalId);
void	deleteVBO(GLuint& vao, GLuint& indexId, GLuint& vertexId, GLuint& normalId,
				  GLuint& coordId);

// Perspective and lookat
// 
// From http://spointeau.blogspot.com/2013/12/hello-i-am-looking-at-opengl-3.html
//
template<class T>
Eigen::Matrix<T, 4, 4> perspective
(
	double fovyR,
	double aspect,
	double zNear,
	double zFar
)
{
	assert(aspect > 0);
	assert(zFar > zNear);

	double	tanHalfFovy = tan(fovyR / 2.0);
	Eigen::Matrix<T, 4, 4>	res = Eigen::Matrix<T, 4, 4>::Zero();
	res(0, 0) = 1.0 / (aspect * tanHalfFovy);
	res(1, 1) = 1.0 / (tanHalfFovy);
	res(2, 2) = -(zFar + zNear) / (zFar - zNear);
	res(3, 2) = -1.0;
	res(2, 3) = -(2.0 * zFar * zNear) / (zFar - zNear);

	return res;
}

template<class T>
Eigen::Matrix<T, 4, 4> lookAt
(
	const Eigen::Matrix<T, 3, 1>& eye,
	const Eigen::Matrix<T, 3, 1>& center,
	const Eigen::Matrix<T, 3, 1>& up
)
{

	Eigen::Matrix<T, 3, 1>	f = (center - eye).normalized();
	Eigen::Matrix<T, 3, 1>	u = up.normalized();
	Eigen::Matrix<T, 3, 1>	s = f.cross(u).normalized();
							u = s.cross(f);
	
	Eigen::Matrix<T, 4, 4>	res;
	res << s.x(), s.y(), s.z(), -s.dot(eye),
		   u.x(), u.y(), u.z(), -u.dot(eye),
		   -f.x(), -f.y(), -f.z(), f.dot(eye),
		   0, 0, 0, 1;

	return res;
}

// From http://en.wikipedia.org/wiki/Orthographic_projection
template<class T>
Eigen::Matrix<T, 4, 4> orthographic
(
	double left,
	double right,
	double bottom,
	double top,
	double near,
	double far
)
{
	assert(far > near);

	Eigen::Matrix<T, 4, 4>	res = Eigen::Matrix<T, 4, 4>::Zero();
	res(0, 0) = 2.0 / (right - left);
	res(1, 1) = 2.0 / (top - bottom);
	res(2, 2) = -2.0 / (far - near);
	res(3, 3) = 1.0;
	res(0, 3) = -(right + left)/(right - left);
	res(1, 3) = -(top + bottom)/(top - bottom);
	res(2, 3) = -(far + near) / (far - near);

	return res;
}

#endif	// __GL_SHADER_H_
