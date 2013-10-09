#ifndef SURFACE_INTEGRATOR_H
#define SURFACE_INTEGRATOR_H

#include "../scene/scene.h"
#include "../sampler/sampler.h"
#include "../math/rng.h"
#include "../material/bsdf.h"

class SurfaceIntegrator
{
public:
	int width , height;

	int samplesPerPixel;

	Scene scene;

	RNG rng;

	virtual void init(char *filename , Parameters& para);

	virtual Color3 raytracing(const Ray& ray , int dep);

	void render(char *filename);
};

#endif