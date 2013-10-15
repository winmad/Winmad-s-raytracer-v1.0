#ifndef VERTEX_KDTREE_H
#define VERTEX_KDTREE_H

#include "../math/color.h"
#include "../math/vector.h"
#include "../material/bsdf.h"
#include <vector>

struct PathVertex
{
	Vector3 hitPos;
	Color3 throughput;
	int length;

	BSDF bsdf;

	Real dVCM , dVC , dVM;
};

struct VertexSorted
{
	Real pos;
	int index;
};

class VertexKDtreeNode
{
public:
	int axis;

	std::vector<PathVertex> vertices;

	VertexSorted *p[3];
	int *div;

	VertexKDtreeNode *left , *right;

	VertexKDtreeNode()
		: left(NULL) , right(NULL) {}
};

class VertexKDtree
{
public:
	int totNum;

	int depMax;

	VertexKDtreeNode *root;

	VertexKDtree()
		: root(NULL) {}

	void init(const std::vector<PathVertex>& vertices);

	void buildTree(PhotonKDtreeNode *tr , int dep);

	void searchVertexInRadius(std::vector<PathVertex>& vertices ,
		PhotonKDtreeNode *tr , const Vector3& pos ,
		Real radius);
};

#endif