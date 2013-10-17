#ifndef SURFACE_INTEGRATOR_H
#define SURFACE_INTEGRATOR_H

#include "../scene/scene.h"
#include "../scene/film.h"
#include "../scene/KDtree.h"
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

	ImageFilm *film;

	virtual void init(char *filename , Parameters& para);

	virtual Color3 raytracing(const Ray& ray , int dep);

	virtual void render();

	virtual void outputImage(char *filename);
};

#endif