#ifndef VOLUME_H
#define VOLUME_H

#include "../math/color.h"
#include "../math/vector.h"
#include "../math/transform.h"
#include "../geometry/geometry.h"

// Phase functions
Real phaseIsotropic(const Vector3& w , const Vector3& wp);
Real phaseRayleigh(const Vector3& w , const Vector3& wp);
Real phaseMieHazy(const Vector3& w , const Vector3& wp);
Real phaseMieMurky(const Vector3& w , const Vector3& wp);
Real phaseHG(const Vector3& w , const Vector3& wp , Real g);
Real phaseSchlick(const Vector3& w , const Vector3& wp , Real g);

//

class Volume
{
public:
	virtual ~Volume();
	virtual AABB worldBound() = 0;
	virtual bool hit(const Ray& ray , Real *t0 , Real *t1) = 0;
	virtual Color3 sigmaA(const Vector3& , const Vector3& , Real time) = 0;
	virtual Color3 sigmaS(const Vector3& , const Vector3& , Real time) = 0;
	virtual Color3 emit(const Vector3& , const Vector3& , Real time) = 0;
	virtual Real p(const Vector3& , const Vector3& , const Vector3& , Real time) = 0;
	virtual Color3 sigmaT(const Vector3& p , const Vector3& wo , Real time);
	virtual Color3 tau(const Ray& ray , Real step = 1.f , Real offset = 0.5f) = 0;
};

class DensityVolume : public Volume
{
public:
	Color3 sigA , sigS , le;
	Real g;
	//Transform worldToVolume;

	DensityVolume() {}

	DensityVolume(const Color3& sa , const Color3& ss , Real g ,
		const Color3& le)
		: sigA(sa) , sigS(ss) , g(g) , le(le) {}

	virtual Real density(const Vector3& pos) = 0;

	Color3 sigmaA(const Vector3& p , const Vector3& , Real time)
	{
		return sigA * density(p);
	}

	Color3 sigmaS(const Vector3& p , const Vector3& , Real time)
	{
		return sigS * density(p);
	}

	Color3 sigmaT(const Vector3& p , const Vector3& wo , Real time)
	{
		return (sigA + sigS) * density(p);
	}

	Color3 emit(const Vector3& p , const Vector3& , Real time)
	{
		return le * density(p);
	}

	Real p(const Vector3& p , const Vector3& w , const Vector3& wp , Real time)
	{
		return phaseHG(w , wp , g);
	}

	Color3 tau(const Ray& ray , Real step /* = 1.f  */, Real offset /* = 0.5f */);
};

#endif