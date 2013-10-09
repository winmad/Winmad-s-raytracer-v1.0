#ifndef SAMPLER_H
#define SAMPLER_H

#include "../math/vector.h"
#include "../scene/scene.h"
#include <vector>

Vector3 sampleTriangle(const Vector3& samples ,
    const Vector3& v1 , const Vector3& v2 ,
    const Vector3& v3);

Vector3 sampleRectangle(const Vector3& samples ,
    const Vector3& v0 , const Vector3& v1 ,
    const Vector3& v2);

Vector3 sampleRectangleStratified(const Vector3& samples ,
    const Vector3& v0 , const Vector3& v1 ,
    const Vector3& v2 , int curLayer , int totLayer);

Vector3 sampleUniformDisk(const Vector3& samples);

Real uniformDiskPdf();

/* Importance sampling: cosine , pdf = cos(theta) / PI */
Vector3 sampleCosHemisphere(const Vector3& samples ,
	Real *pdf);

Real cosHemispherePdf(const Vector3& n , const Vector3& dir);

Vector3 samplePowerCosHemisphere(const Vector3& samples ,
	Real power , Real *pdf);

Real powerCosHemispherePdf(const Vector3& n , const Vector3& dir ,
	Real power);

/* Uniform sampling , pdf = 1 / (4 * PI) */
Vector3 sampleUniformSphere(const Vector3& samples , Real *pdf);

Real uniformSpherePdf();

#endif
