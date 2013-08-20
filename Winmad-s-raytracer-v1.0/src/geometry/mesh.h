#ifndef MESH_H
#define MESH_H

#include "../math/vector.h"
#include "triangle.h"
#include <vector>

class MeshTriangle
{
public:
	int v[3];

	MeshTriangle() {}
	
	MeshTriangle(int v0 , int v1 , int v2)
	{
		v[0] = v0; v[1] = v1; v[2] = v2;
	}
};

class Mesh 
{
public:
	std::vector<Vector3> vertices;
	std::vector<MeshTriangle> triangles;

	Material material;

	Mesh() {}

	~Mesh() {}

	void load(const char *filename);
};

#endif