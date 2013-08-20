#include "phong.h"

Color3 Phong::calcBrdf(const Vector3& wi , 
	const Vector3& wo , const Vector3& n)
{
	Vector3 reflectDir = getReflectDir(wi , n);
	Real alpha = reflectDir ^ wo;
	Color3 res = kd / PI;
	res = res + ks * ((PHONG_POWER_INDEX + 2) / 2 / PI * 
		pow(clampVal(alpha , 0.0 , 1.0) , PHONG_POWER_INDEX));
	return res;
}

Color3 Phong::calcBtdf(const Vector3& wi ,
	const Vector3& wo , const Vector3& n)
{
	Vector3 otherHemisphereDir = n * (wi ^ (-n)) * 2.0 + wi;
	otherHemisphereDir.normalize();
	return calcBrdf(otherHemisphereDir , wo , n);
}