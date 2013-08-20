#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "AABB.h"
#include "intersection.h"
#include "../material/material.h"
#include "../math/vector.h"

class Geometry
{
public:
	AABB box;

	Geometry() {}
	virtual ~Geometry();
	virtual void setBox();
	virtual Material& getMaterial();
	virtual bool hit(const Ray& ray , Intersection& inter);
};

#endif