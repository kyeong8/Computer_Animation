
#version 400

layout (location = 0) in vec3	VertexPosition;
layout (location = 1) in vec3	VertexNormal;

out vec3	position;
out vec3	normal;

// Transformation matrices: GLSL employ column-major matrices.
uniform mat4	ModelViewProjectionMatrix;
uniform mat4	ModelViewMatrix;
uniform mat3	NormalMatrix;	// Transpose of the inverse of modelViewMatrix

// Twisting 
uniform	float	twisting = 0.0;

void
main(void)
{
	// the twisting angle is proportional to the distance from the origin.
	float	angle = twisting * length(VertexPosition.xy);
	float	cosLength = cos(angle);
	float	sinLength = sin(angle);

	// New position due to twisting
	float	x = cosLength * VertexPosition.x - sinLength * VertexPosition.y;
	float	y = sinLength * VertexPosition.x + cosLength * VertexPosition.y;
	vec4	newVertexPosition = vec4(x, y, VertexPosition.z, 1.0);

	// New normal due to twisting
	x = cosLength * VertexNormal.x - sinLength * VertexNormal.y;
	y = sinLength * VertexNormal.x + cosLength * VertexNormal.y;
	vec3	newVertexNormal = vec3(x, y, VertexNormal.z);

	// Position for the rasterization
	gl_Position = ModelViewProjectionMatrix * newVertexPosition;

	// Position and normal in the view coordinate system for the fragment shader
	position = vec3(ModelViewMatrix * newVertexPosition);
	normal = normalize(NormalMatrix * newVertexNormal);	
}
