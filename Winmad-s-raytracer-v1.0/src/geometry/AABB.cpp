#include "AABB.h"

bool AABB::inside(const Vector3& p)
{
	return (p.x >= l.x && p.x <= r.x &&
		p.y >= l.y && p.y <= r.y && p.z >= l.z && p.z <= r.z);
}

bool AABB::hit(const Ray& ray , Real& t1 , Real& t2)
{
	Real tmin = -INF , tmax = INF;

	for (int i = 0; i < 3; i++)
	{
		Real invRayDir = 1.f / ray.dir[i];
		Real tNear = (l[i] - ray.origin[i]) * invRayDir;
		Real tFar = (r[i] - ray.origin[i]) * invRayDir;

		if (tNear > tFar)
		{
			std::swap(tNear , tFar);
		}

		tmin = std::max(tmin , tNear);
		tmax = std::min(tmax , tFar);
		if (tmin > tmax)
			return 0;
	}
	t1 = tmin;
	t2 = tmax;
	return 1;
}
