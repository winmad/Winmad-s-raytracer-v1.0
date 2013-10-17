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
	int pathLength;

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

	void buildTree(VertexKDtreeNode *tr , int dep);

	void destroy(VertexKDtreeNode *tr);

	template<typename tQuery>
	void searchVertexInRadius(VertexKDtreeNode *tr , const Vector3& pos ,
		Real radius , tQuery& query)
	{
		if (tr == NULL)
			return;
		if (tr->vertices.size() <= 0)
			return;

		VertexKDtreeNode *near , *far;
		Real p_pos = get_projection(pos , tr->axis);
		Real split_pos = get_projection(tr->vertices[0].hitPos , tr->axis);
		Real delta = p_pos - split_pos;

		if (cmp(delta) <= 0)
		{
			near = tr->left;
			far = tr->right;
		}
		else
		{
			near = tr->right;
			far = tr->left;
		}

		searchVertexInRadius(near , pos , radius , query);

		Vector3 d = pos - tr->vertices[0].hitPos;
		Real dis = d.length();

		if (dis < radius)
		{
			query.process(tr->vertices[0]);
		}
		if (std::abs(delta) < radius)
			searchVertexInRadius(far , pos , radius , query);
	}
};

static Real get_projection(const Vector3& v , int axis)
{
	if (axis == 0)
		return v.x;
	if (axis == 1)
		return v.y;
	if (axis == 2)
		return v.z;
	return INF;
}

static int cmp_sort_point(const void *a , const void *b)
{
	VertexSorted p1 = *(VertexSorted*)a;
	VertexSorted p2 = *(VertexSorted*)b;
	return cmp(p1.pos - p2.pos);
}

#endif