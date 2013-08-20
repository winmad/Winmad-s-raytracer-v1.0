#ifndef SPHERE_H
#define SPHERE_H

#include "../math/vector.h"
#include "geometry.h"
#include "AABB.h"

class Sphere : public Geometry
{
public:
	Vector3 center;
	Real radius;

	Material material;

	void setBox()
	{
		box = AABB(center.x - radius , center.x + radius ,
			center.y - radius , center.y + radius ,
			center.z - radius , center.z + radius);
	}

	Sphere() {}

	Sphere(Vector3 c , Real r)
		: center(c) , radius(r)
	{
		setBox();
	}

	Sphere(const Sphere& s)
	{
		*this = s;
	}
	
	~Sphere() {}

	Material& getMaterial();

	bool hit(const Ray& ray , Intersection& inter);
};

#endif