
#version 400

layout (location = 0) in vec3	VertexPosition;
layout (location = 1) in vec3	VertexNormal;

out vec3	position;
out vec3	normal;

// Transformation matrices: GLSL employ column-major matrices.
uniform mat4	ModelViewProjectionMatrix;
uniform mat4	ModelViewMatrix;
uniform mat3	NormalMatrix;	// Transpose of the inverse of modelViewMatrix

void
main(void)
{
	gl_Position = ModelViewProjectionMatrix * vec4(VertexPosition, 1.0);

	// View coordinate system
	position = vec3(ModelViewMatrix * vec4(VertexPosition, 1.0));
	normal = normalize(NormalMatrix * VertexNormal);	
}

