#include "homogeneous.h"

AABB HomogeneousVolumeDensity::worldBound()
{
	return box;
}

bool HomogeneousVolumeDensity::hit(const Ray& ray , Real *t0 , Real *t1)
{
	if (t0 != NULL && t1 != NULL)
		return box.hit(ray , *t0 , *t1);
	else
	{
		Real a , b;
		return box.hit(ray , a , b);
	}
}

Color3 HomogeneousVolumeDensity::sigmaA(const Vector3& p , const Vector3& , Real time)
{
	return box.inside(p) ? sigA : Color3(0.f);
}

Color3 HomogeneousVolumeDensity::sigmaS(const Vector3& p , const Vector3& , Real time)
{
	return box.inside(p) ? sigS : Color3(0.f);
}

Color3 HomogeneousVolumeDensity::sigmaT(const Vector3& p , const Vector3& wo , Real time)
{
	return box.inside(p) ? (sigA + sigS) : Color3(0.f);
}

Color3 HomogeneousVolumeDensity::emit(const Vector3& p , const Vector3& , Real time)
{
	return box.inside(p) ? le : Color3(0.f);
}

Real HomogeneousVolumeDensity::p(const Vector3& p , const Vector3& wi , const Vector3& wo , Real time)
{
	return box.inside(p) ? phaseHG(wi , wo , g) : 0.f;
}

Color3 HomogeneousVolumeDensity::tau(const Ray& ray , Real step /* = 1.f  */, Real offset /* = 0.5f */)
{
	Real t0 , t1;
	if (!hit(ray , &t0 , &t1))
		return Color3(0.f);

	Vector3 d = ray(t1) - ray(t0);
	Real dist = d.length();
	return (sigA + sigS) * dist;
}