#include "singleScattering.h"

void SingleScattering::init(char *filename , Parameters& para)
{
	minPathLength = 0;
	maxPathLength = 10;
	iterations = 1;

	stepSize = 1.f;

	scene.init(filename , para);

	height = para.HEIGHT; width = para.WIDTH;

	film = new ImageFilm(height , width);
}

static FILE *fp = fopen("debug_vss.txt" , "w");

void SingleScattering::render()
{
	for (int iter = 0; iter < iterations; iter++)
		runIteration(iter);
}

void SingleScattering::outputImage(char *filename)
{
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < i; j++)
		{
			Color3 tmp = film->color[i][j];
			film->color[i][j] = film->color[j][i];
			film->color[j][i] = tmp;
			/*
			tmp = film->color[i][j];
			fprintf(fp , "c(%d,%d)=(%.3f,%.3f,%.3f)\n" , i , j ,
				tmp.r , tmp.g , tmp.b);
			*/
		}
	}
	film->outputImage(filename , 1.f / iterations , 2.2);
}

void SingleScattering::runIteration(int iter)
{
	cameraPathNum = height * width;

	// generating camera paths
	for (int index = 0; index < cameraPathNum; index++)
	{
		int pathIndex = index % (height * width);

		SSPathState cameraState;
		Vector3 screenSample = generateCameraSample(pathIndex , cameraState);

		Color3 color(0);

		for (;; cameraState.pathLength++)
		{
			Ray ray(cameraState.origin + cameraState.dir * EPS ,
				cameraState.dir);

			Intersection inter;

			if (scene.intersect(ray , inter) == NULL)
			{
				break;
			}

			ray.tmin = 0.f;
			ray.tmax = inter.t;

			Vector3 hitPos = inter.p;

			BSDF bsdf(-ray.dir , inter , scene);
			if (!bsdf.isValid())
				break;

			Color3 li(0);

			if (inter.matId < 0)
			{
				AbstractLight *light = scene.lights[-inter.matId - 1];

				if (cameraState.pathLength >= minPathLength &&
					cameraState.specularPath)
				{
					Ray lightRay(cameraState.origin , ray.dir , ray.tmin ,
						ray.tmax);
					color = color + (cameraState.throughput |
						getLightRadiance(light , cameraState , 
						hitPos , ray.dir) | transmittance(lightRay));
				}
				break;
			}

			if (cameraState.pathLength >= maxPathLength)
				break;
            
			// vertex connection: connect to light source
			if (!bsdf.isDelta)
			{
				if (cameraState.pathLength + 1 >= minPathLength)
				{
					li = li + (cameraState.throughput |
						getDirectIllumination(cameraState , hitPos , bsdf));
				}
			}

			Color3 t;
			Color3 lvi = getSingleScattering(ray , t);
			color = color + (li | t) + lvi;
			//color = color + li;

			if (!sampleScattering(bsdf , hitPos , cameraState))
				break;
		}

		film->addColor((int)screenSample.x , (int)screenSample.y , color);
	}
}

Color3 SingleScattering::transmittance(const Ray& ray)
{
	if (!scene.volume)
		return Color3(1.f);
	Real step , offset;
	step = stepSize;
	offset = rng.randFloat();
	Color3 tau = scene.volume->tau(ray , step , offset);
	return exp(-tau);
}

Color3 SingleScattering::getSingleScattering(const Ray& ray , Color3& t)
{
	Volume *vr = scene.volume;
	Real t0 , t1;
	if (!vr || !vr->hit(ray , &t0 , &t1) || (t1 - t0) == 0.f)
	{
		t = Color3(1.f);
		return Color3(0.f);
	}

	Color3 res(0.f);
	int nSamples = (int)std::ceil((t1 - t0) / stepSize);
	Real step = (t1 - t0) / nSamples;
	Color3 tr(1.f);

	Vector3 p = ray(t0) , pPrev;
	Vector3 w = -ray.dir;
	t0 += rng.randFloat() * step;

	for (int i = 0; i < nSamples; i++ , t0 += step)
	{
		pPrev = p;
		p = ray(t0);
		Ray tauRay(pPrev , p - pPrev);
		Color3 stepTau = vr->tau(ray , 0.5f * stepSize , rng.randFloat());
		tr = (tr | exp(-stepTau));

		if (tr.intensity() < 1e-3)
		{
			Real contProb = 0.5f;
			if (rng.randFloat() > contProb)
			{
				tr = Color3(0.f);
				break;
			}
			tr = tr / contProb;
		}

		res = res + (tr | vr->emit(p , w , 0.f));

		Color3 ss = vr->sigmaS(p , w , 0.f);

		if (!ss.isBlack() && scene.lights.size() > 0)
		{
			int lightCount = scene.lights.size();
			int lightId = (int)(rng.randFloat() * lightCount);
			AbstractLight *light = scene.lights[lightId];

			Vector3 dirToLight;
			Real dist , directPdf , emissionPdf , cosAtLight;

			Color3 illu = light->illuminance(scene.sceneSphere , p ,
				rng.randVector3() , dirToLight , dist , directPdf , &emissionPdf ,
				&cosAtLight);

			if (!illu.isBlack() && directPdf > 0.f &&
				!scene.occluded(p , dirToLight , p + dirToLight * dist))
			{
				Ray lightRay(p + dirToLight * dist , -dirToLight);
				Color3 ls = (illu | transmittance(lightRay));
				Real phaseTerm = vr->p(p , w , dirToLight , 0.f);
				res = res + (tr | ss | ls) * phaseTerm *
					(Real)lightCount / directPdf;
			}
		}
	}

	t = tr;
	return res * step;
}

bool SingleScattering::sampleScattering(BSDF& bsdf , 
	const Vector3& hitPos , SSPathState& pathState)
{
	Real bsdfDirPdf , cosWo;
	int sampledBSDFType;
	Color3 bsdfFactor = bsdf.sample(scene , rng.randVector3() ,
		pathState.dir , bsdfDirPdf , cosWo , &sampledBSDFType);

	if (bsdfFactor.isBlack())
		return 0;

	Real bsdfRevPdf = bsdfDirPdf;
	if ((sampledBSDFType & BSDF_SPECULAR) == 0)
		bsdfRevPdf = bsdf.pdf(scene , pathState.dir , 1);

	Real contProb = bsdf.continueProb;
	if (rng.randFloat() > contProb)
		return 0;

	bsdfDirPdf *= contProb;
	bsdfRevPdf *= contProb;

	// Partial sub-path MIS quantities
	// the evaluation is completed when the actual hit point is known!
	// i.e. after tracing the ray, out of the procedure
	if (sampledBSDFType & BSDF_SPECULAR)
	{
		pathState.specularVertexNum++;
	}
	else
	{
		pathState.specularPath &= 0;
	}

	pathState.origin = hitPos;
	pathState.throughput = (pathState.throughput | bsdfFactor) *
		(cosWo / bsdfDirPdf);

	return 1;
}

Vector3 SingleScattering::generateCameraSample(const int pathIndex , 
	SSPathState& cameraState)
{
	Camera& camera = scene.camera;
	int y = pathIndex % width;
	int x = pathIndex / width;

	Vector3 jitter = rng.randVector3();
	Vector3 sample = Vector3((Real)x + jitter.x , 
		(Real)y + jitter.y , 0.f);

	Ray ray = camera.generateRay(sample.x , sample.y);

	Real cosAtCamera = camera.forward ^ ray.dir;
	Real imagePointToCameraDist = camera.imagePlaneDist /
		cosAtCamera;
	Real imageToSolidAngleFactor = SQR(imagePointToCameraDist) /
		cosAtCamera;

	Real cameraPdf = imageToSolidAngleFactor;

	cameraState.origin = ray.origin;
	cameraState.dir = ray.dir;

	cameraState.pathLength = 1;
	cameraState.specularPath = 1;
	cameraState.specularVertexNum = 0;

	cameraState.throughput = Color3(1);
	
	return sample;
}

Color3 SingleScattering::getLightRadiance(AbstractLight *light , 
	SSPathState& cameraState , const Vector3& hitPos , 
	const Vector3& rayDir)
{
	int lightCount = scene.lights.size();
	Real lightPickProb = 1.f / lightCount;

	Real directPdfArea , emissionPdf;
	Color3 radiance = light->getRadiance(scene.sceneSphere ,
		rayDir , hitPos , &directPdfArea , &emissionPdf);

	Color3 res(0);

	if (radiance.isBlack())
		return res;

	if (cameraState.pathLength == 1)
		return radiance;

	directPdfArea *= lightPickProb;
	emissionPdf *= lightPickProb;

	return radiance;
}

Color3 SingleScattering::getDirectIllumination(SSPathState& cameraState , 
	const Vector3& hitPos , BSDF& bsdf)
{
	Color3 res(0);

	int lightCount = scene.lights.size();
	Real lightPickProb = 1.f / lightCount;

	int lightId = (int)(rng.randFloat() * lightCount);
	AbstractLight *light = scene.lights[lightId];

	Vector3 dirToLight;
	Real dist , directPdf , emissionPdf , cosAtLight , cosAtSurface;
	int sampledBSDFType;

	Color3 illu = light->illuminance(scene.sceneSphere ,
		hitPos , rng.randVector3() , dirToLight , dist ,
		directPdf , &emissionPdf , &cosAtLight);

	Real bsdfDirPdf , bsdfRevPdf , cosToLight;

	if (!illu.isBlack() && directPdf > 0)
	{
		Color3 bsdfFactor = bsdf.f(scene , dirToLight ,
			cosToLight , &bsdfDirPdf , &bsdfRevPdf);

		Color3 tmp;

		if (!bsdfFactor.isBlack())
		{	
			Real contProb = bsdf.continueProb;

			bsdfDirPdf *= light->isDelta() ? 0.f : contProb;
			bsdfRevPdf *= contProb;
            
			Ray lightRay(hitPos , dirToLight , 0.f , dist);
			tmp = (illu | bsdfFactor | transmittance(lightRay)) * 
				cosToLight / (directPdf * lightPickProb);

			if (!tmp.isBlack() && !scene.occluded(hitPos , dirToLight ,
				hitPos + dirToLight * dist))
			{
				//fprintf(fp , "weight = %.6f\n" , weight);

				if (light->isDelta())
				{
					res = res + tmp;
				}
				else
				{
					Real _weight = mis(directPdf) / 
						(mis(directPdf) + mis(bsdfDirPdf));

					res = res + tmp * _weight;
				}
			}
		}
	}

	if (!light->isDelta())
	{
		Color3 bsdfFactor = bsdf.sample(scene , rng.randVector3() ,
			dirToLight , directPdf , cosAtSurface , &sampledBSDFType);

		if (!bsdfFactor.isBlack() && directPdf > 0)
		{
			Real weight = 1.f;

			Real lightPdf;
			if (!(sampledBSDFType & BSDF_SPECULAR))
			{
				illu = light->getRadiance(scene.sceneSphere , dirToLight , hitPos ,
					&lightPdf , &emissionPdf);

				if (cmp(lightPdf) == 0)
					return res;

				weight = mis(directPdf) /
					(mis(directPdf) + mis(lightPdf));
			}

			Intersection lightInter;
			Color3 tmp(0.f);
			Ray ray(hitPos + dirToLight * EPS , dirToLight);

			if (scene.intersect(ray , lightInter) != NULL)
			{
				if (lightInter.matId < 0)
				{
					if (light != scene.lights[-lightInter.matId - 1])
					{
						illu = Color3(0.f);
					}
				}
				else
				{
					illu = Color3(0.f);
				}
			}
			else
			{
				illu = Color3(0.f);
				if (scene.background != NULL)
				{
					illu = getLightRadiance(scene.background ,
							cameraState , Vector3(0) , ray.dir);
				}
			}

			if (!illu.isBlack())
			{
				ray.tmin = 0.f;
				ray.tmax = lightInter.t;

				tmp = (illu | bsdfFactor | transmittance(ray)) * 
					cosAtSurface / (directPdf);
				res = res + tmp * weight;
			}
		}
	}
    
	return res;
}