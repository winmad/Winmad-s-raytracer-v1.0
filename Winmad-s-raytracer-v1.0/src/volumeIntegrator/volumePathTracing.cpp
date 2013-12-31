#include "volumePathTracing.h"
#include <stack>

void VolumePathTracing::init(char *filename , Parameters& para)
{
	minPathLength = 0;
	maxPathLength = 10;
	iterations = 5000;

	stepSize = 1.f;

	scene.init(filename , para);

	height = para.HEIGHT; width = para.WIDTH;

	film = new ImageFilm(height , width);
}

static FILE *fp = fopen("debug_vpt.txt" , "w");

void VolumePathTracing::render()
{
	for (int iter = 0; iter < iterations; iter++)
		runIteration(iter);
}

void VolumePathTracing::outputImage(char *filename)
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

void VolumePathTracing::runIteration(int iter)
{
	cameraPathNum = height * width;

	// generating camera paths
	for (int index = 0; index < cameraPathNum; index++)
	{
		int pathIndex = index % (height * width);

		VptPathState cameraState;
		Vector3 screenSample = generateCameraSample(pathIndex , cameraState);
		
		std::stack<Volume*> vStack;
		vStack.push(NULL);

		if (scene.volume != NULL && 
			scene.volume->worldBound().inside(cameraState.origin))
		{
			cameraState.vr = scene.volume;
			vStack.push(cameraState.vr);
		}

		Color3 color(0);

		for (;; cameraState.pathLength++)
		{
			Ray ray(cameraState.origin + cameraState.dir * EPS ,
				cameraState.dir);

			Intersection inter;

			Geometry *surface = scene.intersect(ray , inter);

			Real t0 , t1;
			bool interMedia = 0;
			if (cameraState.vr)
				interMedia = cameraState.vr->hit(ray , &t0 , &t1);
			
			if (!interMedia)
			{
				if (surface == NULL)
				{
					break;
				}
				else
				{
					bool contFlag = 0;
					int inOutFlag = 0;
					color = color + handleSurface(cameraState , ray , 
						inter , contFlag , inOutFlag);
					if (!contFlag)
						break;

					if (inOutFlag == 1)
					{
						cameraState.vr = NULL;
						vStack.push(NULL);
					}
					else if (inOutFlag == 2)
					{
						vStack.pop();
						cameraState.vr = vStack.top();
					}
				}
			}
			else
			{
				Real marchLen;
				Color3 tr;
				Vector3 pos , dir;
				Real cmf;
				cmf = getMediaScattering(cameraState.vr , ray , t0 , t1 , marchLen ,
					pos , dir , tr);

				if (marchLen >= inter.t)
				{
					bool contFlag = 0;
					int inOutFlag = 0;
					color = color + handleSurface(cameraState , ray , 
						inter , contFlag , inOutFlag);
					if (!contFlag)
						break;

					if (inOutFlag == 1)
					{
						cameraState.vr = NULL;
						vStack.push(NULL);
					}
					else if (inOutFlag == 2)
					{
						vStack.pop();
						cameraState.vr = vStack.top();
					}
				}
				else
				{
					if (rng.randFloat() <= 2) // <= cmf
					{
						color = color + handleVolume(cameraState , ray , t0 , t1 ,
							tr , pos , dir , cmf);
						if (cameraState.pathLength > maxPathLength)
							break;
					}
					else if (surface != NULL)
					{
						bool contFlag = 0;
						int inOutFlag = 0;
						color = color + handleSurface(cameraState , ray , 
							inter , contFlag , inOutFlag);
						if (!contFlag)
							break;

						if (inOutFlag == 1)
						{
							cameraState.vr = NULL;
							vStack.push(NULL);
						}
						else if (inOutFlag == 2)
						{
							vStack.pop();
							cameraState.vr = vStack.top();
						}
					}
					else
						break;
				}
			}
		}
		film->addColor((int)screenSample.x , (int)screenSample.y , color);
	}
}

Color3 VolumePathTracing::handleSurface(VptPathState& cameraState , 
	Ray& ray , Intersection& inter , bool& contFlag , int& inOutFlag)
{
	contFlag = 0;
	Vector3 hitPos = inter.p;
	Color3 res(0);

	BSDF bsdf(-ray.dir , inter , scene);
	if (!bsdf.isValid())
		return res;
	
	ray.tmin = 0.f;
	ray.tmax = inter.t;

	Color3 tr = transmittance(cameraState.vr , ray);

	cameraState.throughput = (cameraState.throughput | tr);

	if (inter.matId < 0)
	{
		AbstractLight *light = scene.lights[-inter.matId - 1];

		if (cameraState.pathLength >= minPathLength &&
			cameraState.specularPath)
		{
			Ray lightRay(hitPos , -ray.dir , 0.f , inter.t);
			res = res + (cameraState.throughput |
				getLightRadiance(light , cameraState , 
				hitPos , ray.dir) | transmittance(cameraState.vr , lightRay));
		}
		return res;
	}

	if (cameraState.pathLength >= maxPathLength)
		return res;

	// vertex connection: connect to light source
	if (!bsdf.isDelta)
	{
		if (cameraState.pathLength + 1 >= minPathLength)
		{
			res = res + (cameraState.throughput |
				getDirectIllumination(cameraState , hitPos , bsdf));
		}
	}

	contFlag = sampleScattering(bsdf , hitPos , cameraState , inOutFlag);
	return res;
}

Color3 VolumePathTracing::handleVolume(VptPathState& cameraState , 
	Ray& ray , Real& t0 , Real& t1 , Color3& tr , Vector3& pos , 
	Vector3& dir , Real& pdf)
{
	Volume *vr = scene.volume;
	Color3 res(0);

	Color3 ss = vr->sigmaS(pos , -ray.dir , 0.f);

	Color3 st = vr->sigmaT(pos , -ray.dir , 0.f);
	Real scatterAlbedo = ss.intensity() / st.intensity();

	if (rng.randFloat() > scatterAlbedo)
	{
		cameraState.pathLength = maxPathLength + 1;
		return res;
	}
	//cameraState.throughput = (cameraState.throughput | tr);

	if (!cameraState.throughput.isBlack() && scene.lights.size() > 0)
	{
		int lightCount = scene.lights.size();
		int lightId = (int)(rng.randFloat() * lightCount);
		AbstractLight *light = scene.lights[lightId];

		Vector3 dirToLight;
		Real dist , directPdf , emissionPdf , cosAtLight;

		Color3 illu = light->illuminance(scene.sceneSphere , pos ,
			rng.randVector3() , dirToLight , dist , directPdf , &emissionPdf ,
			&cosAtLight);

		if (!illu.isBlack() && directPdf > 0.f &&
			!scene.occluded(pos , dirToLight , pos + dirToLight * dist))
		{
			Ray lightRay(pos + dirToLight * dist , -dirToLight , 0.f , dist);
			Color3 ls = (illu | transmittance(cameraState.vr , lightRay));
			Real phaseTerm = vr->p(pos , -ray.dir , dirToLight , 0.f);
			res = res + (cameraState.throughput | ss | ls) * phaseTerm *
				(Real)lightCount / directPdf;
		}
	}

	cameraState.specularPath &= 0;
	cameraState.origin = pos;
	cameraState.dir = dir;

	return res;
}

Color3 VolumePathTracing::transmittance(Volume *vr , const Ray& ray)
{
	if (vr == NULL)
		return Color3(1.f);
	Real step , offset;
	step = stepSize;
	offset = rng.randFloat();
	Color3 tau = vr->tau(ray , step , offset);
	return exp(-tau);
}

Real VolumePathTracing::getMediaScattering(Volume* vr , Ray& ray , Real& t0 , 
	Real& t1 , Real& marchLen , Vector3& pos , Vector3& dir , Color3& tr)
{
	vr = scene.volume;

	marchLen = sampleSegment(rng.randFloat() , 
		vr->sigmaT(ray.origin , -ray.dir , 0.f).intensity() , t1 - t0);
	pos = ray.origin + ray.dir * t0 + ray.dir * marchLen;
	dir = samplePhaseHG(-ray.dir , 0.f , rng.randVector3());

	Ray marchRay(ray.origin , ray.dir , 0.f , t0 + marchLen);
	tr = transmittance(vr , marchRay);

	return 1 - std::exp(-vr->sigmaT(ray.origin , -ray.dir , 0.f).intensity() *
		(t1 - t0));
}

// 0 = normal, 1 = enter in, 2 = exit out
bool VolumePathTracing::sampleScattering(BSDF& bsdf , 
	const Vector3& hitPos , VptPathState& pathState , int& inOutFlag)
{
	Real bsdfDirPdf , cosWo;
	int sampledBSDFType;
	Color3 bsdfFactor = bsdf.sample(scene , rng.randVector3() ,
		pathState.dir , bsdfDirPdf , cosWo , &sampledBSDFType);
	inOutFlag = 0;

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

	if (sampledBSDFType & BSDF_TRANSMISSION)
	{
		if ((pathState.dir ^ bsdf.localFrame.normal()) >= 0)
			inOutFlag = 2;
		else
			inOutFlag = 1;
	}

	pathState.origin = hitPos;
	pathState.throughput = (pathState.throughput | bsdfFactor) *
		(cosWo / bsdfDirPdf);

	return 1;
}

Vector3 VolumePathTracing::generateCameraSample(const int pathIndex , 
	VptPathState& cameraState)
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

Color3 VolumePathTracing::getLightRadiance(AbstractLight *light , 
	VptPathState& cameraState , const Vector3& hitPos , 
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

Color3 VolumePathTracing::getDirectIllumination(VptPathState& cameraState , 
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
			tmp = (illu | bsdfFactor | transmittance(cameraState.vr ,
				lightRay)) * cosToLight / (directPdf * lightPickProb);

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

				tmp = (illu | bsdfFactor | transmittance(cameraState.vr ,
					ray)) * cosAtSurface / (directPdf);
				res = res + tmp * weight;
			}
		}
	}
    
	return res;
}
