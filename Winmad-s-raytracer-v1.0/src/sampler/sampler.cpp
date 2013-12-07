#include "sampler.h"

Vector3 sampleTriangle(const Vector3& samples ,
    const Vector3& v1 , const Vector3& v2 ,
    const Vector3& v3)
{
    Vector3 p1 = v2 - v1;
    Vector3 p2 = v3 - v1;
	Real u1 = sqrt(samples.x);
    Real beta = 1.f - u1;
    Real gamma = samples.y * u1;
    return v1 + p1 * beta + p2 * gamma;
}

Vector3 sampleRectangle(const Vector3& samples ,
    const Vector3& v0 , const Vector3& v1 ,
    const Vector3& v2)
{
    Vector3 p1 = v1 - v0;
    Vector3 p2 = v2 - v0;

    Real a = samples.x;
    Real b = samples.y;

    return v0 + p1 * a + p2 * b;
}

Vector3 sampleRectangleStratified(const Vector3& samples ,
    const Vector3& v0 , const Vector3& v1 ,
    const Vector3& v2 , int curLayer , int totLayer)
{
    Vector3 p1 = v1 - v0;
    Vector3 p2 = v2 - v0;
    int len = (int)sqrt((double)totLayer);
    int row = curLayer / len;
    int col = curLayer % len;

    Real a = (samples.x + row) / (Real)len;
    Real b = (samples.y + col) / (Real)len;

    return v0 + p1 * a + p2 * b;
}

Vector3 sampleUniformDisk(const Vector3& samples) 
{
	Real phi , r;

	Real a = 2 * samples.x - 1;   /* (a,b) is now on [-1,1]^2 */
	Real b = 2 * samples.y - 1;

	if (a > -b)      /* region 1 or 2 */
	{
		if (a > b)   /* region 1, also |a| > |b| */
		{
			r = a;
			phi = (PI / 4.f) * (b / a);
		}
		else        /* region 2, also |b| > |a| */
		{
			r = b;
			phi = (PI / 4.f) * (2.f - (a / b));
		}
	}
	else            /* region 3 or 4 */
	{
		if(a < b)   /* region 3, also |a| >= |b|, a != 0 */
		{
			r = -a;
			phi = (PI / 4.f) * (4.f + (b / a));
		}
		else        /* region 4, |b| >= |a|, but a==0 and b==0 could occur. */
		{
			r = -b;

			if (b != 0)
				phi = (PI / 4.f) * (6.f - (a / b));
			else
				phi = 0;
		}
	}

	Vector3 res;
	res.x = r * std::cos(phi);
	res.y = r * std::sin(phi);
	res.z = 0.f;
	return res;
}

Real uniformDiskPdf()
{
	return INV_PI;
}

/* Importance sampling: cosine , pdf = cos(theta) / PI */
Vector3 sampleCosHemisphere(const Vector3& samples , Real *pdf)
{
    Real u1 = 2.f * PI * samples.x;
	Real u2 = std::sqrt(1.f - samples.y);

	Vector3 res(std::cos(u1) * u2 , std::sin(u1) * u2 , 
		std::sqrt(samples.y));

	if (pdf)
		*pdf = res.z * INV_PI;

	res.normalize();
	return res;
}

Real cosHemispherePdf(const Vector3& n , const Vector3& dir)
{
	return clampVal(n ^ dir , 0.f , 1.f) * INV_PI;
}

Vector3 samplePowerCosHemisphere(const Vector3& samples , 
	Real power , Real *pdf)
{
	Real u1 = 2.f * PI * samples.x;
	Real u2 = std::pow(samples.y , 1.f / (power + 1.f));
	Real u3 = std::sqrt(1.f - u2 * u2);

	if (pdf)
		*pdf = (power + 1.f) * std::pow(u2 , power) * (0.5f * INV_PI);

	Vector3 res(std::cos(u1) * u3 , std::sin(u1) * u3 ,
		u2);
	res.normalize();
	return res;
}

Real powerCosHemispherePdf(const Vector3& n , const Vector3& dir , 
	Real power)
{
	Real cos = clampVal(n ^ dir , 0.f , 1.f);
	return (power + 1.f) * std::pow(cos , power) * (0.5f * INV_PI);
}

Vector3 sampleUniformSphere(const Vector3& samples , Real *pdf)
{
    Real u1 = samples.x;
    Real u2 = samples.y;
    Real z = 1.f - 2.f * u1;
    Real r = sqrt(std::max(0.f , 1.f - z * z));
    Real phi = 2.f * PI * u2;
    Real x = r * cos(phi);
    Real y = r * sin(phi);

	if (pdf)
		*pdf = INV_PI * 0.25f;

    Vector3 res(x , y , z);
	res.normalize();
	return res;
}

Real uniformSpherePdf()
{
	return INV_PI * 0.25f;
}

Vector3 sampleUniformHemisphere(const Vector3& samples , Real *pdf)
{
	Vector3 res = sampleUniformSphere(samples , pdf);
	res.z = std::abs(res.z);
	return res;
}

Real uniformHemispherePdf()
{
	return INV_PI * 0.5f;
}