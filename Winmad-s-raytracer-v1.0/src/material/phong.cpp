#include "phong.h"
#include "../sampler/sampler.h"

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

Color3 Phong::calcRho(const Vector3& wo , const Vector3 &n)
{
	Color3 res = Color3(0.0 , 0.0 , 0.0);
	int sampleNum = 10;
	for (int i = 0; i < sampleNum; i++)
	{
		Vector3 wi = sampleDirOnHemisphere(n);
		wi.normalize();
		//Real invPdf = PI / (wi ^ n);
		//res = res + (calcBrdf(wi , wo , n) * (wi ^ n) * invPdf);
		res = res + calcBrdf(wi , wo , n) * PI;
	}
	return res / sampleNum;
}