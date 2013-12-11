#include "sphere.h"

int Sphere::getMatId()
{
	return matId;
}

Vector3 Sphere::samplePos(const Vector3& samples , Vector3& normal)
{
	Real theta = samples.x * PI;
	Real phi = samples.y * PI;
	normal = Vector3(std::sin(theta) * std::cos(phi) , 
		std::sin(theta) * std::sin(phi) , std::cos(theta));
	return center + normal * radius;
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

	if (inter.t < ray.tmin ||
		inter.t > ray.tmax)
	{
		inter.t = INF;
		return 0;
	}

	inter.p = ray(inter.t);
	inter.n = inter.p - center;
	inter.n.normalize();
	inter.matId = matId;
	return 1;
}