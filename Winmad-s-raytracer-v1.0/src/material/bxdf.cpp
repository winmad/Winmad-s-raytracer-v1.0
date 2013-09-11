#include "bxdf.h"

Color3 BxDF::calcBrdf(const Vector3& wi , 
	const Vector3& wo , const Vector3& n)
{
	return Color3(0.0 , 0.0 , 0.0);
}

Color3 BxDF::calcBtdf(const Vector3& wi ,
	const Vector3& wo , const Vector3& n)
{
	return Color3(0.0 , 0.0 , 0.0);
}

Color3 BxDF::calcRho(const Vector3& wo ,
	const Vector3& n)
{
	return Color3(0.0 , 0.0 , 0.0);
}