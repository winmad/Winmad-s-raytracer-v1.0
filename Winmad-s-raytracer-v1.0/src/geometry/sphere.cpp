#include "sphere.h"

Material& Sphere::getMaterial()
{
	return material;
}

bool Sphere::hit(const Ray& ray , Intersection& inter)
{
	Real tmp1 , tmp2;
 	if (!box.hit(ray , tmp1 , tmp2))
 	{
 		inter.t = INF;
 		return 0;
 	}

	Vector3 oc = center - ray.origin;
	Real l_oc = oc ^ oc;
	Real t_ca = oc ^ ray.dir;
	if (cmp(t_ca) < 0)
	{
		inter.t = INF;
		return 0;
	}
	Real t_hc = SQR(radius) - l_oc + SQR(t_ca);
	if (cmp(t_hc) <= 0)
	{
		inter.t = INF;
		return 0;
	}
	Real t1 , t2 , d = sqrt(t_hc);
	t1 = t_ca - d; t2 = t_ca + d;
	if (cmp(t2) <= 0)
	{
		inter.t = INF;
		return 0;
	}
	else
	{
		if (cmp(t1) <= 0)
		{
			inter.t = t2;
			inter.inside = 1;
		}
		else
		{
			inter.t = t1;
			inter.inside = 0;
		}
	}

	if (cmp(inter.t - ray.tmin) < 0 ||
		cmp(inter.t - ray.tmax) > 0)
	{
		inter.t = INF;
		return 0;
	}

	inter.p = ray(inter.t);
	inter.n = inter.p - center;
	inter.n.normalize();
	return 1;
}