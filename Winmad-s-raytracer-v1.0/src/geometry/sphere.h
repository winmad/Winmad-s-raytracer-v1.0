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
	int matId;

	void setBox()
	{
		box = AABB(center.x - radius , center.x + radius ,
			center.y - radius , center.y + radius ,
			center.z - radius , center.z + radius);
	}

	Sphere() {}

	Sphere(Vector3 c , Real r , int matId)
		: center(c) , radius(r) , matId(matId)
	{
		setBox();
	}

	Sphere(const Sphere& s)
	{
		*this = s;
	}
	
	~Sphere() {}

	int getMatId();

	Real getArea()
	{
		return 4 * PI * SQR(radius);
	}

	bool hit(const Ray& ray , Intersection& inter);

	Vector3 samplePos(const Vector3& samples , Vector3& normal);
};

#endif