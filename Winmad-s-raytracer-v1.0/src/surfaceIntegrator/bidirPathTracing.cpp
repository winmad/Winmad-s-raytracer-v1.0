#include "bidirPathTracing.h"

static FILE *fp = fopen("debug_bpt.txt" , "w");

void BidirPathTracing::init(char *filename , Parameters& para)
{
	minPathLength = 0;
	maxPathLength = 10;
	iterations = 1;

	samplesPerPixel = para.SAMPLES_PER_PIXEL;

	scene.init(filename , para);

	height = para.HEIGHT; width = para.WIDTH;
	pixelNum = height * width;

	film = new ImageFilm(height , width);
}

void BidirPathTracing::render()
{
	for (int iter = 0; iter < iterations; iter++)
		runIteration(iter);
}

void BidirPathTracing::outputImage(char *filename)
{
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < i; j++)
		{
			Color3 tmp = film->color[i][j];
			film->color[i][j] = film->color[j][i];
			film->color[j][i] = tmp;

			tmp = film->color[i][j];
			/*
			fprintf(fp , "c(%d,%d)=(%.3f,%.3f,%.3f)\n" , i , j ,
				tmp.r , tmp.g , tmp.b);
			*/
		}
	}
	film->outputImage(filename , 1.f / iterations , 2.2);
}

static Real mis(Real pdf)
{
	return pdf;
}

void BidirPathTracing::runIteration(int iter)
{
	lightPathNum = height * width;
	cameraPathNum = lightPathNum;

	lightStateIndex.resize(lightPathNum);
	memset(&lightStateIndex[0] , 0 , lightStateIndex.size() * sizeof(int));

	lightStates.reserve(lightPathNum);
	lightStates.clear();

	cameraStates.reserve(cameraPathNum);
	cameraStates.clear();

	// generating light paths
	for (int pathIndex = 0; pathIndex < lightPathNum; pathIndex++)
	{
		BidirPathState lightState;

		bool deltaLight;
		generateLightSample(lightState);

		int s = 1;

		for (;; lightState.pathLength++ , s++)
		{
			Ray ray(lightState.origin + lightState.dir * EPS ,
				lightState.dir);
			Intersection inter;
			if (scene.intersect(ray , inter) == NULL)
				break;

			Vector3 hitPos = inter.p;

			BSDF bsdf(-ray.dir , inter , scene);
			if (!bsdf.isValid())
				break;

			lightState.pos = hitPos;
			lightState.bsdf = bsdf;

			if (lightState.pathLength > 1 || lightState.isFiniteLight)
			{
				lightState.dVCM *= mis(SQR(inter.t));
			}
			lightState.dVCM /= mis(std::abs(bsdf.cosWi()));
			lightState.dVC /= mis(std::abs(bsdf.cosWi()));
			
			if (!bsdf.isDelta)
				lightStates.push_back(lightState);

			// connect to camera
			if (!bsdf.isDelta)
			{
				if (lightState.pathLength + 1 >= minPathLength)
				{
					Vector3 imagePos = scene.camera.worldToRaster.tPoint(hitPos);
					if (scene.camera.checkRaster(imagePos.x , imagePos.y))
					{
						Color3 res = connectToCamera(lightState , hitPos , bsdf);

						// weight
						//Real weight = 1.f / (lightState.pathLength + 1 - lightState.specularVertexNum);

						film->addColor((int)imagePos.x , (int)imagePos.y , res);
					}
				}
			}

			if (lightState.pathLength + 2 > maxPathLength)
				break;

			if (!sampleScattering(bsdf , hitPos , lightState))
				break;
		}

		lightStateIndex[pathIndex] = (int)lightStates.size();
	}

	// generating camera paths
	for (int index = 0; index < cameraPathNum; index++)
	{
		int pathIndex = index % (height * width);

		BidirPathState cameraState;
		Vector3 screenSample = generateCameraSample(pathIndex , cameraState);

		Color3 color(0);

		for (;; cameraState.pathLength++)
		{
			Ray ray(cameraState.origin + cameraState.dir * EPS ,
				cameraState.dir);

			Intersection inter;

			if (scene.intersect(ray , inter) == NULL)
			{
				if (scene.background != NULL)
				{
					if (cameraState.pathLength >= minPathLength)
					{
						// weight
						Real weight = 1.f / (cameraState.pathLength - cameraState.specularVertexNum);

						color = color + (cameraState.throughput |
							getLightRadiance(scene.background ,
							cameraState , Vector3(0) , ray.dir)) * weight;
					}
				}
				break;
			}

			Vector3 hitPos = inter.p;

			BSDF bsdf(-ray.dir , inter , scene);
			if (!bsdf.isValid())
				break;

			cameraState.dVCM *= mis(SQR(inter.t));
			cameraState.dVCM /= mis(std::abs(bsdf.cosWi()));
			cameraState.dVC /= mis(std::abs(bsdf.cosWi()));

			if (inter.matId < 0)
			{
				AbstractLight *light = scene.lights[-inter.matId - 1];

				if (cameraState.pathLength >= minPathLength)
				{
					// weight
					//Real weight = 1.f / (cameraState.pathLength - cameraState.specularVertexNum);

					color = color + (cameraState.throughput |
						getLightRadiance(light , cameraState , 
						hitPos , ray.dir));
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
					// weight
					Real weight = 1.f / (cameraState.pathLength + 1.f - cameraState.specularVertexNum);

					color = color + (cameraState.throughput |
						getDirectIllumination(cameraState , hitPos , bsdf)) *
						weight;
				}
			}

			// vertex connection: connect to light vertices
			if (!bsdf.isDelta)
			{
				int st , ed;
				if (pathIndex == 0)
					st = 0;
				else
					st = lightStateIndex[pathIndex - 1];
				ed = lightStateIndex[pathIndex];

				for (int i = st; i < ed; i++)
				{
					BidirPathState& lightState = lightStates[i];

					if (lightState.pathLength + 1 + 
						cameraState.pathLength < minPathLength)
						continue;

					if (lightState.pathLength + 1 +
						cameraState.pathLength > maxPathLength)
						break;
					
					if (lightState.bsdf.isDelta)
						continue;

					Color3 tmp = connectVertices(lightState ,
						bsdf , hitPos , cameraState);

					// weight
					Real weight = 1.f / (lightState.pathLength + 1.f +
						cameraState.pathLength - lightState.specularVertexNum - 
						cameraState.specularVertexNum);

					color = color + (cameraState.throughput |
						lightState.throughput | tmp) * weight;
				}
			}

			if (!sampleScattering(bsdf , hitPos , cameraState))
				break;
		}

		film->addColor((int)screenSample.x , (int)screenSample.y , color);
	}
}

void BidirPathTracing::generateLightSample(BidirPathState& lightState)
{

	int lightNum = scene.lights.size();
	Real lightPickProb = 1.f / lightNum;

	int lightId = (int)(rng.randFloat() * lightNum);
	AbstractLight *light = scene.lights[lightId];

	Real emissionPdf , directPdf , cosAtLight;
	lightState.throughput = light->emit(scene.sceneSphere ,
		rng.randVector3() , rng.randVector3() , 
		lightState.origin , lightState.dir , emissionPdf ,
		&directPdf , &cosAtLight);

	emissionPdf *= lightPickProb;
	directPdf *= lightPickProb;

	lightState.throughput = lightState.throughput / emissionPdf;
	lightState.pathLength = 1;
	lightState.isFiniteLight = light->isFinite();
	lightState.specularPath = 1;
	lightState.specularVertexNum = 0;

	lightState.throughput = lightState.throughput;

	lightState.dVCM = mis(directPdf / emissionPdf);
	if (!light->isDelta())
	{
		lightState.dVC = mis(1.f / emissionPdf);
	}
	else
	{
		lightState.dVC = 0.f;
	}
}

Color3 BidirPathTracing::connectToCamera(BidirPathState& lightState , 
	const Vector3& hitPos , BSDF& bsdf)
{
	Color3 res(0);

	Camera& camera = scene.camera;
	Vector3 dirToCamera = camera.pos - hitPos;

	if (((-dirToCamera) ^ camera.forward) <= 0)
		return res;

	Real distEye2 = dirToCamera.sqrLength();
	Real dist = std::sqrt(distEye2);
	dirToCamera = dirToCamera / dist;

	Real cosToCamera , bsdfDirPdf , bsdfRevPdf;

	Color3 bsdfFactor = bsdf.f(scene , dirToCamera , cosToCamera ,
		&bsdfDirPdf , &bsdfRevPdf);

	if (bsdfFactor.isBlack())
		return res;

	bsdfRevPdf *= bsdf.continueProb;

	Real cosAtCamera = ((-dirToCamera) ^ camera.forward);
	Real imagePointToCameraDist = camera.imagePlaneDist / cosAtCamera;
	Real imageToSolidAngleFactor = SQR(imagePointToCameraDist) / cosAtCamera;
	Real imageToSurfaceFactor = imageToSolidAngleFactor * std::abs(cosToCamera) / distEye2;

	Real cameraPdfA = imageToSurfaceFactor /* * 1.f */; // pixel area is 1
	
	Real surfaceToImageFactor = 1.f / imageToSurfaceFactor;

	// We divide the contribution by surfaceToImageFactor to convert the (already
	// divided) pdf from surface area to image plane area, w.r.t. which the
	// pixel integral is actually defined. We also divide by the number of samples
	// this technique makes
	res = (lightState.throughput | bsdfFactor) /
		(lightPathNum * surfaceToImageFactor);
    
    if (res.isBlack())
		return res;

	if (scene.occluded(hitPos , dirToCamera , camera.pos))
		return Color3(0);

	Real wLight = mis(cameraPdfA / lightPathNum) * (lightState.dVCM + 
		mis(bsdfRevPdf) * lightState.dVC);

	Real weight = 1.f / (wLight + 1.f);

	//fprintf(fp , "weight = %.6f\n" , weight);

	return res * weight;
}

bool BidirPathTracing::sampleScattering(BSDF& bsdf , 
	const Vector3& hitPos , BidirPathState& pathState)
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

		pathState.dVCM = 0.f;
		pathState.dVC *= mis(cosWo);
	}
	else
	{
		pathState.specularPath &= 0;

		pathState.dVC = mis(1.f / bsdfDirPdf) * (pathState.dVCM +
			pathState.dVC * mis(bsdfRevPdf));
		pathState.dVCM = mis(1.f / bsdfDirPdf);
	}

	pathState.origin = hitPos;
	pathState.throughput = (pathState.throughput | bsdfFactor) *
		(cosWo / bsdfDirPdf);

	return 1;
}

Vector3 BidirPathTracing::generateCameraSample(const int pathIndex , 
	BidirPathState& cameraState)
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

	cameraState.dVCM = mis(lightPathNum / cameraPdf);
	cameraState.dVC = 0.f;
	
	return sample;
}

Color3 BidirPathTracing::getLightRadiance(AbstractLight *light , 
	BidirPathState& cameraState , const Vector3& hitPos , 
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

 	Real wCamera = mis(directPdfArea) * cameraState.dVCM + 
 		mis(emissionPdf) * cameraState.dVC;

	Real weight = 1.f / (1.f + wCamera);

	return radiance * weight;
}

Color3 BidirPathTracing::getDirectIllumination(BidirPathState& cameraState , 
	const Vector3& hitPos , BSDF& bsdf)
{
	Color3 res(0);

	Real weight = 0.f;

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
            
			tmp = (illu | bsdfFactor) * cosToLight / (directPdf * lightPickProb);

			if (!tmp.isBlack() && !scene.occluded(hitPos , dirToLight ,
				hitPos + dirToLight * dist))
			{
				Real wLight = mis(bsdfDirPdf) / mis(directPdf * lightPickProb);
				Real wCamera = mis(emissionPdf * cosToLight / (directPdf * cosAtLight)) *
					(cameraState.dVCM + mis(bsdfRevPdf) * cameraState.dVC);
				weight = 1.f / (wLight + 1.f + wCamera);

				fprintf(fp , "weight = %.6f\n" , weight);

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
				tmp = (illu | bsdfFactor) * cosAtSurface /
					(directPdf);
				res = res + tmp * weight;
			}
		}
	}
    
	return res * weight;
}

// we force to connect eye path to light path 
Color3 BidirPathTracing::connectVertices(BidirPathState& lightState , 
	BSDF& cameraBsdf , const Vector3& hitPos , 
	BidirPathState& cameraState)
{
	Vector3 dir = lightState.pos - hitPos;
	Real dist2 = dir.sqrLength();
	Real dist = std::sqrt(dist2);
	dir = dir / dist;

	Color3 res(0);

	Real cosAtCamera , cameraBsdfDirPdf , cameraBsdfRevPdf;
	Color3 cameraBsdfFactor = cameraBsdf.f(scene , dir , cosAtCamera ,
		&cameraBsdfDirPdf , &cameraBsdfRevPdf);

	if (cameraBsdfFactor.isBlack())
		return res;

	Real cameraContProb = cameraBsdf.continueProb;
	cameraBsdfDirPdf *= cameraContProb;
	cameraBsdfRevPdf *= cameraContProb;

	Real cosAtLight , lightBsdfDirPdf , lightBsdfRevPdf;
	Color3 lightBsdfFactor = lightState.bsdf.f(scene , -dir , cosAtLight ,
		&lightBsdfDirPdf , &lightBsdfRevPdf);

	if (lightBsdfFactor.isBlack())
		return res;

	Real lightContProb = lightState.bsdf.continueProb;
	lightBsdfDirPdf *= lightContProb;
	lightBsdfRevPdf *= lightContProb;

	Real geometryTerm = cosAtLight * cosAtCamera / dist2;

	if (cmp(geometryTerm) < 0)
		return res;

	Real cameraBsdfDirPdfArea = pdfWtoA(cameraBsdfDirPdf , dist , cosAtLight);
	Real lightBsdfDirPdfArea = pdfWtoA(lightBsdfDirPdf , dist , cosAtCamera);

	res = (cameraBsdfFactor | lightBsdfFactor) * geometryTerm;

	if (res.isBlack() || scene.occluded(hitPos , dir , 
		hitPos + dir * dist))
		return Color3(0);

	Real wLight = mis(cameraBsdfDirPdfArea) * 
		(lightState.dVCM + mis(lightBsdfRevPdf) * lightState.dVC);
	Real wCamera = mis(lightBsdfDirPdfArea) *
		(cameraState.dVCM + mis(cameraBsdfRevPdf) * cameraState.dVC);
	Real weight = 1.f / (wLight + 1.f + wCamera);

	return res * weight;
}