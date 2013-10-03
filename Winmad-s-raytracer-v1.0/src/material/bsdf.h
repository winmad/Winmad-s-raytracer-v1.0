#ifndef BSDF_H
#define BSDF_H

#include "../math/color.h"
#include "../math/vector.h"
#include "../math/frame.h"
#include "../geometry/intersection.h"

enum BSDFType
{
	BSDF_REFLECTION = 1 << 0,
	BSDF_TRANSMISSION = 1 << 1,

	BSDF_DIFFUSE = 1 << 2,
	BSDF_GLOSSY = 1 << 3,
	
	BSDF_SPECULAR = BSDF_REFLECTION |
					BSDF_TRANSMISSION,

	BSDF_NON_SPECULAR = BSDF_DIFFUSE |
						BSDF_GLOSSY,

	BSDF_ALL = BSDF_SPECULAR | 
			   BSDF_NON_SPECULAR
};

class BSDF
{
public:
	BSDFType type;

	int matId;

	Frame local;

	Vector3 wiLocalFix;

	bool isDelta; // true if material is purely specular

	Real continueProb; // used for Russian roulette

	Real fresnelReflect; // Fresnel reflection coefficient

	BSDF() : matId(-1) {}

	BSDF(BxDFType t) 
		: type(t) {}

	bool MatchType(BxDFType t)
	{
		return (type & t) == type;
	}

	bool hasComponent(BxDFType t)
	{
		return (type & t) == t;
	}

	virtual Color3 calcBrdf(const Vector3& wi , 
		const Vector3& wo , const Vector3& n);

	virtual Color3 calcBtdf(const Vector3& wi ,
		const Vector3& wo , const Vector3& n);

	virtual Color3 calcRho(const Vector3& wo , 
		const Vector3 &n);
};

#endif