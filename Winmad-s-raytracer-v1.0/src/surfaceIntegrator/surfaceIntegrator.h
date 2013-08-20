#ifndef SURFACE_INTEGRATOR_H
#define SURFACE_INTEGRATOR_H

#include "../scene/scene.h"
#include "../sampler/sampler.h"

class SurfaceIntegrator
{
public:
	int width , height;
	ViewPort viewPort;

	int samplesPerPixel;

	Scene scene;

	virtual void init(char *filename , Parameters& para);

	virtual Color3 raytracing(const Ray& ray , int dep);

	void render(char *filename);
};

#endif