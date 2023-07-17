#ifndef _MESH_H_
#define _MESH_H_

#include <Eigen/Dense>
using namespace Eigen;

#include <vector>
using namespace std;

int readMesh(const char* fname, MatrixXf& vertex, MatrixXf& normal, ArrayXXi& face);
int readMesh(const char* fname, MatrixXf& vertex, ArrayXXi& face,
				MatrixXf& faceNormal, MatrixXf& normal);

#endif	// _MESH_H_