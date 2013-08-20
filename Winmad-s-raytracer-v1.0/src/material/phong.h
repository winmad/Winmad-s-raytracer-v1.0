#ifndef PHONG_H
#define PHONG_H

#include "bxdf.h"

class Phong : public BxDF
{
public:
	static const int PHONG_POWER_INDEX = 10;

	Phong() {}

	Color3 calcBrdf(const Vector3& wi , 
		const Vector3& wo , const Vector3& n);

	Color3 calcBtdf(const Vector3& wi ,
		const Vector3& wo , const Vector3& n);
};

#endif