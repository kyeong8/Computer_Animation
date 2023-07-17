
#ifdef _WIN32
#define _CRT_SECURE_NO_WARNINGS		// fopen instead of fopen_s
#endif

#include "glShader.h"

#include <iostream>
using namespace std;


// Shader functions
//
bool
isOK(const char* message, const char* file, int line, bool exitOnError, bool report)
{
	GLenum	errorCode = glGetError();
	if (errorCode != GL_NO_ERROR)
	{
		if (report)
		{
			cerr << "OpenGL: ";
			if (file)		cerr << file;
			if (line != -1) cerr << ":" << line;
			if (message)	cerr << " " << message;
			cerr << " " << gluErrorString(errorCode) << endl;
		}

		if (exitOnError)	exit(errorCode);

		return false;
	}

	return true;
}

char*
readShader(const char* filename)
{
	if (filename == NULL)
	{
		cerr << "ERROR: Fail in readShader(" << filename << ")" << endl;
		return NULL;
	}

	FILE* fp = fopen(filename, "r");
	if (fp == NULL)
	{
		cerr << "ERROR: Fail in readShader(" << filename << ")" << endl;
		return NULL;
	}

	fseek(fp, 0, SEEK_END);
	int count = ftell(fp);
	rewind(fp);

	char* content = NULL;
	if (count > 0)
	{
		content = new char[count + 1];		// +1 for null termination
		count = fread(content, sizeof(char), count, fp);
		content[count] = 0;					// Null-termination
	}
	fclose(fp);

	return content;
}

void
printShaderInfoLog(GLuint obj, const char* shaderFilename)
{
	int infoLogLength;
	glGetShaderiv(obj, GL_INFO_LOG_LENGTH, &infoLogLength);
	if (infoLogLength == 0) return;

	// Report the error
	char* infoLog = new char[infoLogLength];
	glGetShaderInfoLog(obj, infoLogLength, NULL, infoLog);

	cerr << "Shader: " << shaderFilename << endl;

	cerr << infoLog;
	delete[]	infoLog;
}

void
printProgramInfoLog(GLuint obj)
{
	int infoLogLength;
	glGetProgramiv(obj, GL_INFO_LOG_LENGTH, &infoLogLength);
	if (infoLogLength == 0) return;

	// Report the error
	char* infoLog = new char[infoLogLength];
	glGetProgramInfoLog(obj, infoLogLength, NULL, infoLog);
	cerr << "Shader Program: " << infoLog;
	delete[]	infoLog;
}

GLuint
createShaderFromFile(GLenum shaderType, const char* filename)
{
	// Create the vertex shader
	GLuint	shader = glCreateShader(shaderType);
	if (isOK("glCreateShader()", __FILE__, __LINE__) == false)	return	0;

	if (shader == 0)
	{
		cerr << "ERROR: Fail in creating the shader for " << filename << endl;
		return 0;
	}

	// Read the shader file into a string
	const char* shaderSource = readShader(filename);
	if (shaderSource == NULL)	return	0;

	// Set the shader source
	glShaderSource(shader, 1, &shaderSource, NULL);

	// Delete the string read from the shader file
	delete[]	shaderSource;

	if (isOK("glShaderSource()", __FILE__, __LINE__) == false)	return	0;

	// Compile the shader
	glCompileShader(shader);
	if (isOK("glCompileShader()", __FILE__, __LINE__) == false)	return	0;

	// Print the compile error if exists
	printShaderInfoLog(shader, filename);

	return	shader;
}

// Create the shaders and the program
void
createShaders(const char* vertexShaderFileName, const char* fragmentShaderFileName,
	GLuint& program, GLuint& vertexShader, GLuint& fragmentShader)
{
	// Create ther vertex and fragment shaders
	vertexShader = createShaderFromFile(GL_VERTEX_SHADER, vertexShaderFileName);
	fragmentShader = createShaderFromFile(GL_FRAGMENT_SHADER, fragmentShaderFileName);

	// Create the program with the vertex and fragment shaders
	program = glCreateProgram();

	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);

	glLinkProgram(program);
	printProgramInfoLog(program);
}

// Delete the shaders and the program
void
deleteShaders(GLuint program, GLuint vertexShader, GLuint fragmentShader)
{
	if (vertexShader)	glDeleteShader(vertexShader);
	if (fragmentShader) glDeleteShader(fragmentShader);
	if (program)		glDeleteShader(program);
}

// Uniform parameter
int
getUniformLocation(GLuint program, const char* name)
{
	GLint loc = glGetUniformLocation(program, name);
	if (isOK("glGetUniformLocation()", __FILE__, __LINE__) == false)	return	-1;

	if (loc < 0)	cerr << "Can't find the uniform parameter " << name << endl;

	return	loc;
}

int
getUniformLocation(GLuint program, const std::string& name)
{
	GLint loc = glGetUniformLocation(program, name.c_str());
	if (isOK("glGetUniformLocation()", __FILE__, __LINE__) == false)	return	-1;

	if (loc < -1)	cerr << "Can't find the uniform parameter " << name << endl;

	return	loc;
}

int
setUniformi(GLuint program, const std::string& name, int i)
{
	GLint location = getUniformLocation(program, name);
	if (location < 0)	return	location;

	glProgramUniform1i(program, location, i);
	if (isOK("setUniform(int)", __FILE__, __LINE__) == false)	return	-1;

	return location;
}

int
setUniform(GLuint program, const std::string& name, float f)
{
	GLint location = getUniformLocation(program, name);
	if (location < 0)	return	location;

	glProgramUniform1f(program, location, f);
	if (isOK("setUniform(float)", __FILE__, __LINE__) == false)	return	-1;
	return location;
}

int
setUniform(GLuint program, const std::string& name, const Vector2f& v)
{
	GLint location = getUniformLocation(program, name);
	if (location < 0)	return	location;

	glProgramUniform2fv(program, location, 1, v.data());
	if (isOK("setUniform()", __FILE__, __LINE__) == false)	return	-1;
	return location;
}

int
setUniform(GLuint program, const std::string& name, const Vector3f& v)
{
	GLint location = getUniformLocation(program, name);
	if (location < 0)	return	location;

	glProgramUniform3fv(program, location, 1, v.data());
	if (isOK("setUniform()", __FILE__, __LINE__) == false)	return	-1;
	return location;
}

int
setUniform(GLuint program, const std::string& name, const Vector4f& v)
{
	GLint location = getUniformLocation(program, name);
	if (location < 0)	return	location;

	glProgramUniform4fv(program, location, 1, v.data());
	if (isOK("setUniform()", __FILE__, __LINE__) == false)	return	-1;
	return location;
}

// Eigen employs column-major matrices.
int
setUniform(GLuint program, const std::string& name, const Matrix3f& m)
{
	GLint location = getUniformLocation(program, name);
	if (location < 0)	return	location;

	glProgramUniformMatrix3fv(program, location, 1, GL_FALSE, m.data());
	if (isOK("setUniform()", __FILE__, __LINE__) == false)	return	-1;
	return location;
}

int
setUniform(GLuint program, const std::string& name, const Matrix4f& m)
{
	GLint location = getUniformLocation(program, name);
	if (location < 0)	return	location;

	glProgramUniformMatrix4fv(program, location, 1, GL_FALSE, m.data());
	if (isOK("setUniform()", __FILE__, __LINE__) == false)	return	-1;
	return location;
}

int
setUniformMatrix3fv(GLuint program, const char* name, const float* value)
{
	GLint location = getUniformLocation(program, name);
	if (location < 0)	return	location;

	glProgramUniformMatrix3fv(program, location, 1, GL_FALSE, value);
	if (isOK("setUniformMatrix3fv()", __FILE__, __LINE__) == false)	return	-1;

	return location;
}

int
setUniformMatrix4fv(GLuint program, const char* name, const float* value)
{
	GLint location = getUniformLocation(program, name);
	if (location < 0)	return	location;

	glProgramUniformMatrix4fv(program, location, 1, GL_FALSE, value);
	if (isOK("setUniformMatrix4fv()", __FILE__, __LINE__, false) == false)	return	-1;

	return location;
}

void
createVBO(GLuint& vao, GLuint& indexId, GLuint& vertexId, GLuint& normalId)
{
	if (indexId == 0)
	{
		// Create VAO
		glGenVertexArrays(1, &vao);

		// Create VBOs
		glGenBuffers(1, &indexId);		// Buffer for triangle indices
		glGenBuffers(1, &vertexId);		// Buffer for vertex positions
		glGenBuffers(1, &normalId);		// Buffer for normal vectors

		isOK("createVBO()", __FILE__, __LINE__);
	}
}

void
createVBO(GLuint& vao, GLuint& idxId, GLuint& vtxId, GLuint& normalId, GLuint& coordId)
{
	if (idxId == 0)
	{
		// Create a new VBO
		glGenVertexArrays(1, &vao);

		glGenBuffers(1, &idxId);		// Buffer for triangle indices
		glGenBuffers(1, &vtxId);		// Buffer for vertex positions
		glGenBuffers(1, &normalId);		// Buffer for normal vectors
		glGenBuffers(1, &coordId);		// Buffer for texture coordinates

		isOK("createVBO()", __FILE__, __LINE__);
	}
}

// Activate the VBO and then upload the mesh data to GPU
int
uploadMesh2VBO(ArrayXXi& face, MatrixXf& vertex, MatrixXf& normal,
	GLuint vao, GLuint indexId, GLuint vertexId, GLuint normalId)
{
	int numTris = face.cols();
	int numVertices = vertex.cols();

	// Activate the VBO and begin the specification of the vertex array
	glBindVertexArray(vao);

	// Bind the client-side memory of the vertex array
	//
	// Index: indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexId);	// Vertex array indices
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, numTris * 3 * sizeof(GLuint), face.data(),
		GL_STATIC_DRAW);

	// Vertex positions
	glBindBuffer(GL_ARRAY_BUFFER, vertexId);	// Vertex position attributes
	glBufferData(GL_ARRAY_BUFFER, numVertices * 3 * sizeof(GLfloat), vertex.data(),
		GL_STATIC_DRAW);

	// Normal vectors
	glBindBuffer(GL_ARRAY_BUFFER, normalId);	// Vertex normal attributes
	glBufferData(GL_ARRAY_BUFFER, numVertices * 3 * sizeof(GLfloat), normal.data(),
		GL_STATIC_DRAW);


	// Layout of the vertex array
	//
	// Vertex positions
	glBindBuffer(GL_ARRAY_BUFFER, vertexId);		// Activate the VBO
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	// Normal vectors
	glBindBuffer(GL_ARRAY_BUFFER, normalId);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	// Deactivate the VBO because the specification has been completed
	glBindVertexArray(0);

	// Check the status
	isOK("uploadMesh2VBO()", __FILE__, __LINE__);

	return numTris;
}

// Activate the VBO and then upload the mesh data to GPU
int
uploadMesh2VBO(ArrayXXi& face, MatrixXf& vertex, MatrixXf& normal, MatrixXf& texture,
	GLuint vao, GLuint indexId, GLuint vertexId, GLuint normalId, GLuint coordId)
{
	int numTris = face.cols();
	int numVertices = vertex.cols();

	// Activate the VBO and begin the specification of the vertex array
	glBindVertexArray(vao);

	// Bind the client-side memory of the vertex array
	//
	// Index: indices
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexId);	// Vertex array indices
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, numTris * 3 * sizeof(GLuint), face.data(),
		GL_STATIC_DRAW);

	// Vertex positions
	glBindBuffer(GL_ARRAY_BUFFER, vertexId);	// Vertex position attributes
	glBufferData(GL_ARRAY_BUFFER, numVertices * 3 * sizeof(GLfloat), vertex.data(),
		GL_STATIC_DRAW);

	// Normal vectors
	glBindBuffer(GL_ARRAY_BUFFER, normalId);	// Vertex normal attributes
	glBufferData(GL_ARRAY_BUFFER, numVertices * 3 * sizeof(GLfloat), normal.data(),
		GL_STATIC_DRAW);

	// Texture coords
	glBindBuffer(GL_ARRAY_BUFFER, coordId);	// Vertex attributes
	glBufferData(GL_ARRAY_BUFFER, numVertices * 2 * sizeof(GLfloat), texture.data(),
		GL_STATIC_DRAW);


	// Layout of the vertex array
	//
	// Vertex positions
	glBindBuffer(GL_ARRAY_BUFFER, vertexId);		// Activate the VBO
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	// Normal vectors
	glBindBuffer(GL_ARRAY_BUFFER, normalId);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	// Texture coords
	glBindBuffer(GL_ARRAY_BUFFER, coordId);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, NULL);

	// Deactivate the VBO because the specification has been completed
	glBindVertexArray(0);

	// Check the status
	isOK("uploadMesh2VBO()", __FILE__, __LINE__);

	return numTris;
}

void
drawVBO(GLuint vao, int numTris)
{
	// Bind the vertex array object
	glBindVertexArray(vao);

	// Draw triangles
	glDrawElements(GL_TRIANGLES, numTris * 3, GL_UNSIGNED_INT, NULL);

	// Break the vertex array object binding
	glBindVertexArray(0);

	// Check to see if there have been errors
	isOK("drawVBO()", __FILE__, __LINE__);
}

void
deleteVBO(GLuint& vao, GLuint& indexId, GLuint& vertexId, GLuint& normalId)
{
	if (indexId != 0)
	{
		// Delete the VBO
		glDeleteVertexArrays(1, &vao);

		glDeleteBuffers(1, &indexId);		// Buffer for triangle indices
		glDeleteBuffers(1, &vertexId);		// Buffer for vertex positions
		glDeleteBuffers(1, &normalId);		// Buffer for texture coordinates

		isOK("deleteVBO()", __FILE__, __LINE__);

		// Invalidate all the Ids
		vao = 0;
		indexId = 0;
		vertexId = 0;
		normalId = 0;
	}
}

void
deleteVBO(GLuint& vao, GLuint& idxId, GLuint& vtxId, GLuint& normalId, GLuint& coordId)
{
	if (idxId != 0)
	{
		// Delete the VBO
		glDeleteVertexArrays(1, &vao);

		glDeleteBuffers(1, &idxId);			// Buffer for triangle indices
		glDeleteBuffers(1, &vtxId);			// Buffer for vertex positions
		glDeleteBuffers(1, &normalId);		// Buffer for texture coordinates
		glDeleteBuffers(1, &coordId);		// Buffer for texture coordinates

		isOK("deleteVBO()", __FILE__, __LINE__);

		// Invalidate all the Ids
		vao = 0;
		idxId = 0;
		vtxId = 0;
		normalId = 0;
		coordId = 0;
	}
}
