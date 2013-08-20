#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "../math/color.h"
#include "../math/vector.h"
#include "../scene/scene.h"
#include "../parameters.h"
#include "surfaceIntegrator.h"

class WhittedIntegrator : public SurfaceIntegrator
{
public:
	int maxTracingDepth;

	WhittedIntegrator() {}

	void init(char *filename , Parameters& para);

	Color3 raytracing(const Ray& ray , int dep);
};

#endif
