#ifndef SAMPLER_H
#define SAMPLER_H

#include "../math/vector.h"
#include "../scene/scene.h"
#include "../math/drand48.h"
#include <vector>

Vector3 sampleOnTriangle(
    const Vector3& v1 , const Vector3& v2 ,
    const Vector3& v3);

Vector3 sampleOnRectangle(
    const Vector3& v0 , const Vector3& v1 ,
    const Vector3& v2);

Vector3 sampleOnRectangleStratified(
    const Vector3& v0 , const Vector3& v1 ,
    const Vector3& v2 , int curLayer , int totLayer);

Vector3 uniformSampleDisk();

/* Importance sampling: cosine , pdf = cos(theta) / PI */
Vector3 sampleDirOnHemisphere(const Vector3& n);

/* Uniform sampling , pdf = 1 / (4 * PI) */
Vector3 uniformSampleDirOnSphere();

std::vector<PointLight> samplePointsOnAreaLight(
    std::vector<Triangle>& triangles , int totSamples);

#endif
