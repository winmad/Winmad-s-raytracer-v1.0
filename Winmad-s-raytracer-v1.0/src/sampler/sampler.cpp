#include "sampler.h"
#include "../math/drand48.h"

Vector3 sampleOnTriangle(
    const Vector3& v1 , const Vector3& v2 ,
    const Vector3& v3)
{
    Vector3 p1 = v2 - v1;
    Vector3 p2 = v3 - v1;
	Real u1 = sqrt(drand48());
    Real beta = 1.0 - u1;
    Real gamma = drand48() * u1;
    return v1 + p1 * beta + p2 * gamma;
}

Vector3 sampleOnRectangle(
    const Vector3& v0 , const Vector3& v1 ,
    const Vector3& v2)
{
    Vector3 p1 = v1 - v0;
    Vector3 p2 = v2 - v0;

    Real a = drand48();
    Real b = drand48();

    return v0 + p1 * a + p2 * b;
}

Vector3 sampleOnRectangleStratified(
    const Vector3& v0 , const Vector3& v1 ,
    const Vector3& v2 , int curLayer , int totLayer)
{
    Vector3 p1 = v1 - v0;
    Vector3 p2 = v2 - v0;
    int len = (int)sqrt((double)totLayer);
    int row = curLayer / len;
    int col = curLayer % len;

    Real a = (drand48() + row) / (Real)len;
    Real b = (drand48() + col) / (Real)len;

    return v0 + p1 * a + p2 * b;
}

Vector3 uniformSampleDisk() 
{
	Real u1 = drand48() , u2 = drand48();
	Real r = sqrt(u1);
	Real theta = 2.0 * PI * u2;
	return Vector3(r * cos(theta) , r * sin(theta) , 0.0);
}

/* Importance sampling: cosine , pdf = cos(theta) / PI */
Vector3 sampleDirOnHemisphere(const Vector3& n)
{
    Vector3 res;
	res = uniformSampleDisk();
	res.z = sqrt(std::max(0.0 , 1.0 - SQR(res.x) - SQR(res.y)));
	return res;
}

Vector3 uniformSampleDirOnSphere()
{
    Real u1 = drand48();
    Real u2 = drand48();
    Real z = 1.0 - 2.0 * u1;
    Real r = sqrt(std::max(0.0 , 1.0 - z * z));
    Real phi = 2.0 * PI * u2;
    Real x = r * cos(phi);
    Real y = r * sin(phi);
    return Vector3(x , y , z);
}

std::vector<PointLight> samplePointsOnAreaLight(
    std::vector<Triangle>& triangles , int totSamples)
{
    std::vector<Real> area;
    Real totArea = 0.0;
    for (int i = 0; i < triangles.size(); i++)
    {
        Vector3 tmp = (triangles[i].p1 - triangles[i].p0) *
            (triangles[i].p2 - triangles[i].p0);
        Real tp = fabs(sqrt(tmp.sqrLength()));
        totArea += tp;
        area.push_back(tp);
    }
    
    std::vector<PointLight> res;
    PointLight l;
    for (int i = 0; i < triangles.size(); i++)
    {
        int k = (int)(area[i] / totArea * totSamples);
        for (int j = 0; j < k; j++)
        {
        	l.pos = sampleOnTriangle(triangles[i].p0 ,
                                             triangles[i].p1 ,
                                             triangles[i].p2);
            l.color = triangles[i].getMaterial().bxdf->kd;
            res.push_back(l);
        }
    }
    return res;
}
