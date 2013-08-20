#ifndef PATH_TRACER_H
#define PATH_TRACER_H

#include "../scene/scene.h"
#include "../parameters.h"
#include "surfaceIntegrator.h"

class PathIntegrator : public SurfaceIntegrator
{
public:
	int maxTracingDepth;
    
    int samplesOfLight;
    
    int samplesOfHemisphere;

	PathIntegrator() {}

	void init(char *filename , Parameters& para);

	Color3 raytracing(const Ray& ray , int dep);
};

#endif
