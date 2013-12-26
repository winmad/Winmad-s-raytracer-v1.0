#ifndef VOLUME_INTEGRATOR_H
#define VOLUME_INTEGRATOR_H

#include "../volume/volume.h"
#include "../scene/scene.h"
#include "../scene/film.h"
#include "../scene/KDtree.h"
#include "../sampler/sampler.h"
#include "../math/rng.h"
#include "../math/color.h"
#include "../math/vector.h"
#include "../material/bsdf.h"
#include <vector>

class VolumeIntegrator
{
public:
	int width , height;

	int samplesPerPixel;

	Scene scene;

	RNG rng;

	ImageFilm *film;

	virtual void init(char *filename , Parameters& para);

	virtual void render();

	virtual void outputImage(char *filename);
};

#endif