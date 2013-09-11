#ifndef BXDF_H
#define BXDF_H

#include "../math/color.h"
#include "../math/vector.h"
#include "../geometry/intersection.h"

enum BxDFType
{
	BSDF_REFLECTION = 1 << 0,
	BSDF_TRANSMISSION = 1 << 1,

	BSDF_DIFFUSE = 1 << 2,
	BSDF_GLOSSY = 1 << 3,
	BSDF_SPECULAR = 1 << 4,

	BSDF_ALL_TYPES = BSDF_DIFFUSE |
					 BSDF_GLOSSY |
					 BSDF_SPECULAR,

	BSDF_ALL_REFLECTION = BSDF_REFLECTION |
						  BSDF_ALL_TYPES,

	BSDF_ALL_TRANSMISSION = BSDF_TRANSMISSION |
							BSDF_ALL_TYPES,

	BSDF_ALL = BSDF_ALL_REFLECTION | 
			   BSDF_ALL_TRANSMISSION
};

class BxDF
{
public:
	BxDFType type;

	Color3 kd , ks;

	BxDF() {}

	BxDF(BxDFType t) 
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