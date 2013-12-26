#ifndef AABB_H
#define AABB_H

#include "ray.h"
#include "../math/vector.h"

class AABB
{
public:
	Vector3 l , r;

	// avoid 2-D box
	void extend()
	{
		if (cmp(l.x - r.x) == 0)
			r.x += 10 * EPS;
		if (cmp(l.y - r.y) == 0)
			r.y += 10 * EPS;
		if (cmp(l.z - r.z) == 0)
			r.z += 10 * EPS;
	}

	AABB() {}

	AABB(Vector3 l , Vector3 r)
		: l(l) , r(r)
	{
		extend();
	}

	AABB(Real xmin , Real xmax , Real ymin , Real ymax ,
		Real zmin , Real zmax)
	{
		l = Vector3(xmin , ymin , zmin);
		r = Vector3(xmax , ymax , zmax);
		extend();
	}

	AABB(const AABB& box)
	{
		*this = box;
	}

	~AABB() {}

	bool hit(const Ray& ray , Real& t1 , Real& t2);

	bool inside(const Vector3& p);
};

#endif