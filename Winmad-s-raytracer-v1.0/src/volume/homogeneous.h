#ifndef HOMOGENEOUS_H
#define HOMOGENEOUS_H

#include "volume.h"

class HomogeneousVolumeDensity : public Volume
{
public:
	Color3 sigA , sigS , le;
	Real g;
	AABB box;

	HomogeneousVolumeDensity() {}
	HomogeneousVolumeDensity(const Color3& sigA , const Color3& sigS ,
		Real g , const Color3& le , const AABB& box)
		: sigA(sigA) , sigS(sigS) , g(g) , le(le) , box(box) {}

	AABB worldBound();
	virtual bool hit(const Ray& ray , Real *t0 , Real *t1);
	virtual Color3 sigmaA(const Vector3& p , const Vector3& , Real time);
	virtual Color3 sigmaS(const Vector3& p , const Vector3& , Real time);
	virtual Color3 emit(const Vector3& p , const Vector3& , Real time);
	virtual Real p(const Vector3& p , const Vector3& wi , const Vector3& wo , Real time);
	virtual Color3 sigmaT(const Vector3& p , const Vector3& wo , Real time);
	virtual Color3 tau(const Ray& ray , Real step = 1.f , Real offset = 0.5f);

};

#endif