#ifndef KDTREE_H
#define KDTREE_H

#include "../geometry/ray.h"
#include "../geometry/geometry.h"
#include "../geometry/AABB.h"
#include <vector>

struct Event
{
	enum EventType
	{
		End = 0 , Planar = 1 , Start = 2
	};

	Real pos;
	EventType type;
	int index;
};

enum DivType
{
	LeftOnly = 0 , RightOnly , Both
};

class KDtreeNode
{
public:
	enum Axes
	{
		X_axis = 0 , Y_axis , Z_axis , No_axis
	};

	int objNum;
	Geometry **objlist;
	AABB box;
	KDtreeNode *left , *right;

	int eventNum[3];
	Event *e[3];
	
	DivType *div;

	Axes axis;
	Real splitPlane;
};

class KDtree
{
public:
	int totObjNum;
	
	int depMax;

	KDtreeNode *root;

	KDtree() {}

	void init(const std::vector<Geometry*>& _objlist);

	void buildTree(KDtreeNode *tr , int dep);

	Geometry* traverse(const Ray& ray , KDtreeNode *tr);
};

// For debug
void print_tree(FILE *fp , KDtreeNode *tr);

#endif