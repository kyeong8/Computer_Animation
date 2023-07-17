#include <fstream>

#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>
#include <vector>
using namespace std;

// Vertex, vertex normal, vertex indices for faces
int
readMesh(const char* filename, MatrixXf& vertex, MatrixXf& normal, ArrayXXi& face)
{
	ifstream	is(filename);
	if (is.fail())	return 0;

	char	magicNumber[256];
	is >> magicNumber;

	// # vertices, # faces, #edges
	int nVertices = 0, nFaces = 0, nEdges = 0;
	is >> nVertices >> nFaces >> nEdges;
	cout << "# vertices = " << nVertices << endl;
	cout << "# faces = " << nFaces << endl;

	// Vertices
	vertex.resize(3, nVertices);
	for (int i = 0; i < nVertices; i++)
		is >> vertex(0, i) >> vertex(1, i) >> vertex(2, i);

	// Normal
	normal.resize(3, nVertices);
	normal.setZero();

	// Faces
	face.resize(3, nFaces); // Only support triangles
	int n;
	for (int i = 0; i < nFaces; i++)
	{
		is >> n >> face(0, i) >> face(1, i) >> face(2, i);
		if (n != 3) cout << "# vertices of the " << i << "-th faces = " << n << endl;

		// Normal vector of the face
		Vector3f	v1 = vertex.col(face(1, i)) - vertex.col(face(0, i));
		Vector3f	v2 = vertex.col(face(2, i)) - vertex.col(face(0, i));
		Vector3f	v = v1.cross(v2).normalized();

		// Add it to the normal vector of each vertex
		normal.col(face(0, i)) += v;
		normal.col(face(1, i)) += v;
		normal.col(face(2, i)) += v;
	}

	// Normalization of the normal vectors
	for (int i = 0; i < nVertices; i++)
		normal.col(i).normalize();

	return nEdges;
}

// Vertex, vertex normal, face normal, vertex indices for faces
int 
readMesh(const char* filename, MatrixXf& vertex, ArrayXXi& face,
			MatrixXf& faceNormal, MatrixXf& normal, vector<vector<int>>& edge)
{
	ifstream	is(filename);
	if (is.fail())	return 0;
	
	char	magicNumber[256];
	is >> magicNumber;

	// # vertices, # faces, # edges
	int nVertices = 0, nFaces = 0, nEdges = 0;
	is >> nVertices >> nFaces >> nEdges;
	cout << "# vertices = " << nVertices << endl;
	cout << "# faces = " << nFaces << endl;

	// Vertices
	vertex.resize(3, nVertices);
	for (int i = 0; i < nVertices; i++)
	{
		is >> vertex(0, i) >> vertex(1, i) >> vertex(2, i);
		vector<int> v;
		edge.push_back(v);
	}

	// Normals
	normal.resize(3, nVertices);
	normal.setZero();

	// Faces
	face.resize(3, nFaces);			// Only supprot triangles
	faceNormal.resize(3, nFaces);

	int n;
	int k;
	for (int i = 0; i < nFaces; i++)
	{
		is >> n >> face(0, i) >> face(1, i) >> face(2, i);
		if (n != 3) cout << "# vertices of the " << i << "-th faces = " << n << endl;

		for (int j = 0; j < 3; j++)
		{
			k = (j + 1) % 3;	// endpoints ot the edge (index j, k) 
			if (face(j, i) > face(k, i) && !count(edge[face(j, i)].begin(), edge[face(j, i)].end(), face(k, i)))	// Small index vertices and non - existent
				edge[face(j, i)].push_back(face(k, i));
			else if (face(k, i) > face(j, i) && !count(edge[face(k, i)].begin(), edge[face(k, i)].end(), face(j, i)))
				edge[face(k, i)].push_back(face(j, i));
		}

		// Normal vector of the face
		Vector3f	v1 = vertex.col(face(1, i)) - vertex.col(face(0, i));
		Vector3f	v2 = vertex.col(face(2, i)) - vertex.col(face(0, i));
		Vector3f	v = v1.cross(v2).normalized();

		// Set the face normal vector
		faceNormal.col(i) = v;

		// Add it to the normal vector of each vertex
		normal.col(face(0, i)) += v;
		normal.col(face(1, i)) += v;
		normal.col(face(2, i)) += v;
	}

	// Normalization of the normal vectors
	for (int i = 0; i < nVertices; i++)
		normal.col(i).normalize();

	return nEdges;
}
