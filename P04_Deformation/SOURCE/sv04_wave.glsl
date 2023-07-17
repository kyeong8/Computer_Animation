
#version 400

layout (location = 0) in vec3	VertexPosition;
layout (location = 1) in vec3	VertexNormal;

out vec3	position;
out vec3	normal;

// Transformation matrices: GLSL employ column-major matrices.
uniform mat4	ModelViewProjectionMatrix;
uniform mat4	ModelViewMatrix;
uniform mat3	NormalMatrix;	// Transpose of the inverse of modelViewMatrix

uniform	float	A = 0.03;		// Amplitude
uniform	float	F = 40.0;		// Spatial frequency
uniform	float	phase = 0.0;	// Phase

void
main(void)
{
	// Vertex position
	float	l = length(VertexPosition.xy);
	float	z = VertexPosition.z + A * sin(F * l + phase);
	vec4	newVertexPosition = vec4(VertexPosition.xy, z, 1.0);

	// Jacobian of the wave deformation
	float	eps = 0.0001;
	mat3	Jt;				// Note that Jt[column][row] in GLSL
	Jt[0][0] = 1; Jt[0][1] = 0; Jt[0][2] = 0;
	Jt[1][0] = 0; Jt[1][1] = 1; Jt[1][2] = 0;
	Jt[2][0] = A * cos(F * l + phase) * F * VertexPosition.x / (l + eps);
	Jt[2][1] = A * cos(F * l + phase) * F * VertexPosition.y / (l + eps);
	Jt[2][2] = 1;

	// Inverse transpose of the Jacobian matrix (already transposed)
	vec3	newVertexNormal = inverse(Jt) * VertexNormal;

	// Position for the rasterization
	gl_Position = ModelViewProjectionMatrix * newVertexPosition;

	// Position and normal in the view coordinate system for the fragment shader
	position = vec3(ModelViewMatrix * newVertexPosition);
	normal = normalize(NormalMatrix * newVertexNormal);	
}
