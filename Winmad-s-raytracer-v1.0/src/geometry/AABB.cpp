#include "AABB.h"

bool AABB::hit(const Ray& ray , Real& t1 , Real& t2)
{
	Real tmin , tmax;
	Real a = 1.0f / ray.dir.x;
	if (cmp(a) >= 0)
	{
		tmin = a * (l.x - ray.origin.x);
		tmax = a * (r.x - ray.origin.x);
	}
	else
	{
		tmin = a * (r.x - ray.origin.x);
		tmax = a * (l.x - ray.origin.x);
	}
	a = 1.0f / ray.dir.y;
	if (cmp(a) >= 0)
	{
		tmin = std::max(tmin , a * (l.y - ray.origin.y));
		tmax = std::min(tmax , a * (r.y - ray.origin.y));
	}
	else
	{
		tmin = std::max(tmin , a * (r.y - ray.origin.y));
		tmax = std::min(tmax , a * (l.y - ray.origin.y));
	}
	a = 1.0f / ray.dir.z;
	if (cmp(a) >= 0)
	{
		tmin = std::max(tmin , a * (l.z - ray.origin.z));
		tmax = std::min(tmax , a * (r.z - ray.origin.z));
	}
	else
	{
		tmin = std::max(tmin , a * (r.z - ray.origin.z));
		tmax = std::min(tmax , a * (l.z - ray.origin.z));
	}

	if (cmp(tmin - tmax) < 0) 
	{
		t1 = tmin;
		t2 = tmax;
		return 1;
	}
	else
		return 0;
}
