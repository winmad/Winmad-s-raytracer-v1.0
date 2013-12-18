#ifndef KDTREE_ACCEL_H
#define KDTREE_ACCEL_H

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

class KDtreeAccelNode
{
public:
	int objNum;
	Geometry **objlist;
	AABB box;
	KDtreeAccelNode *left , *right;

	int eventNum[3];
	Event *e[3];
	
	DivType *div;

	int axis;
	Real splitPlane;
};

struct KDTodo
{
	KDtreeAccelNode *node;
	Real tmin , tmax;
};

class KDtreeAccel
{
public:
	int totObjNum;
	
	int depMax;

	KDtreeAccelNode *root;
	
	KDTodo *todo;

	KDtreeAccel() {}

	void init(const std::vector<Geometry*>& _objlist);

	void buildTree(KDtreeAccelNode *tr , int dep);

	Geometry* traverse(const Ray& ray , KDtreeAccelNode *tr);
};

// For debug
void print_tree(FILE *fp , KDtreeAccelNode *tr);

#endif