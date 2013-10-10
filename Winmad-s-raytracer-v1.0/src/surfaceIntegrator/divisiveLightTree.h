#ifndef DIVISIVE_LIGHT_TREE
#define DIVISIVE_LIGHT_TREE

#include "../scene/scene.h"
#include <vector>
#include <queue>

struct VirtualLight
{
	Vector3 p;
	Color3 alpha;
	Real intensity;
	
	VirtualLight() {}
	
	VirtualLight(const Vector3& _p , const Color3& _alpha)
		: p(_p) , alpha(_alpha) 
	{
		intensity = alpha.intensity();
	}
};

struct LightPosSorted
{
	Real pos;
	int index;
};

struct LightTreeNode
{
	int axis;

	VirtualLight vl;

	Real totIntensity;

	AABB box;

	std::vector<VirtualLight> lights;

	LightPosSorted *p[3];
	int *div;

	LightTreeNode *left , *right;

	LightTreeNode()
		: left(NULL) , right(NULL) {}
};

struct LightCluster
{
	LightTreeNode *tr;
	VirtualLight vl;
	AABB box;
	Color3 contrib;
	Real maxErrorRadiance;
	
	LightCluster() {}

	LightCluster(LightTreeNode *_tr)
	{
		tr = _tr;
		vl = tr->vl;
		box = tr->box;
	}

	bool operator <(const LightCluster& c) const
	{
		//return (cmp(maxErrorRadiance - c.maxErrorRadiance) < 0);
		return maxErrorRadiance < c.maxErrorRadiance;
	}
};

class LightTree
{
public:
	const static int MAX_CUT_SIZE = 500;

	LightTreeNode *root;

	int maxCutSize;

	LightTree() : root(NULL) {}

	void init(const std::vector<VirtualLight>& lights);

	void buildTree(LightTreeNode *tr);

	Color3 findLightCuts(const Vector3& p , const Vector3& n ,
		const Vector3& wo , Geometry *g , Scene& scene ,
		std::priority_queue<LightCluster>& cut);
};

#endif