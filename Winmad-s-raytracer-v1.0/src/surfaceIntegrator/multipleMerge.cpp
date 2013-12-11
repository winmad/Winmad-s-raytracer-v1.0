#include "multipleMerge.h"

static FILE *fp = fopen("debug_mm.txt" , "w");

void MultipleMerge::init(char *filename , Parameters& para)
{
	minPathLength = 0;
	maxPathLength = 10;
	iterations = 10;

	samplesPerPixel = para.SAMPLES_PER_PIXEL;

	scene.init(filename , para);

	baseRadius = 0.003f * scene.sceneSphere.sceneRadius;
	radiusAlpha = 0.75f;

	height = para.HEIGHT; width = para.WIDTH;
    pixelNum = height * width;
    
	film = new ImageFilm(height , width);
}

void MultipleMerge::render()
{
	for (int iter = 0; iter < iterations; iter++)
		runIteration(iter);
}

void MultipleMerge::outputImage(char *filename)
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

	film->outputImage(filename , 1.f / iterations , 2.2f);
}

void MultipleMerge::preparation(double mergeRadius)
{
	lightPathNum = height * width;
	interPathNum = 0;

	partialPathNum = lightPathNum;

	radius = mergeRadius;
	Real radiusSqr = SQR(radius);

	mergeKernel = 1.f / (PI * radiusSqr * partialPathNum);

	lightSubPaths.clear();

	// generating light paths
	for (int pathIndex = 0; pathIndex < lightPathNum; pathIndex++)
	{
		MMPathState lightState;
		generateLightSample(lightState);

		for (;; lightState.pathLength++)
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

			if (!bsdf.isDelta)
			{
				// store subpath data
				lightSubPaths.push_back(lightState);
			}

			if (lightState.pathLength + 2 > maxPathLength)
				break;

			Real pdf = 0.f;
			if (!sampleScattering(bsdf , hitPos , lightState , &pdf , 0))
				break;

			Real weightFactor = connectFactor(pdf) / (connectFactor(pdf) +
				mergeFactor());

			lightState.throughput = lightState.throughput * weightFactor;
			lightState.dirContrib = lightState.dirContrib * weightFactor;
		}
	}

	// generating intermediate path
	for (int pathIndex = 0; pathIndex < interPathNum; pathIndex++)
	{
		MMPathState interState;
		generateInterSample(interState);

		for (;; interState.pathLength++)
		{
			Ray ray(interState.origin + interState.dir * EPS ,
				interState.dir);
			Intersection inter;
			if (scene.intersect(ray , inter) == NULL)
				break;

			Vector3 hitPos = inter.p;

			BSDF bsdf(-ray.dir , inter , scene);
			if (!bsdf.isValid())
				break;

			interState.pos = hitPos;
			interState.bsdf = bsdf;

			if (!bsdf.isDelta)
			{
				// store subpath data
				lightSubPaths.push_back(interState);
			}

			if (interState.pathLength + 2 > maxPathLength)
				break;

			Real pdf = 0.f;
			if (!sampleScattering(bsdf , hitPos , interState , &pdf , 0))
				break;

			Real weightFactor = connectFactor(pdf) / (connectFactor(pdf) +
				mergeFactor());

			interState.throughput = interState.throughput * weightFactor;
		}
	}


	// multiple merge between light paths and intermediate paths

	std::vector<Color3> contribs;
	contribs.resize(lightSubPaths.size());

	int mergeIterations = 0;

	for (int mergeIter = 0; mergeIter < mergeIterations; mergeIter++)
	{
		lightTree = new KdTree<MMPathState>(lightSubPaths);

		for (int i = 0; i < lightSubPaths.size(); i++)
		{
			MergeQuery query(*this , lightSubPaths[i]);

			lightTree->searchInRadius(0 , query.lightSubPath.posAtOrigin , 
				radius , query);

			//fprintf(fp , "%d\n" , query.mergeNum);

			Color3 color = (lightSubPaths[i].throughput | query.contrib);

			contribs[i] = color;
		}

		for (int i = 0; i < lightSubPaths.size(); i++)
			lightSubPaths[i].indirContrib = contribs[i];

		delete lightTree;
	}
}

void MultipleMerge::runIteration(int iter)
{
	cameraPathNum = height * width;

	radius = baseRadius;
	radius /= std::pow((Real)(iter + 1) , 0.5f * (1.f - radiusAlpha));
	radius = std::max(radius , EPS);
	Real radiusSqr = SQR(radius);

	preparation(radius);

	mergeKernel = 1.f / (PI * radiusSqr * partialPathNum);

	lightTree = new KdTree<MMPathState>(lightSubPaths);

	//debug
// 	for (int i = 0; i < lightSubPaths.size(); i++)
// 	{
// 		MMPathState& subPath = lightSubPaths[i];
// 		fprintf(fp , "dirC=(%.4f,%.4f,%.4f),indirC=(%.4f,%.4f,%.4f)\n" ,
// 			subPath.dirContrib.r , subPath.dirContrib.g , subPath.dirContrib.b ,
// 			subPath.indirContrib.r , subPath.indirContrib.g , subPath.indirContrib.b);
// 	}

	for (int i = 0; i < lightSubPaths.size(); i++)
	{
		MMPathState& lightState = lightSubPaths[i];
		if (lightState.pathLength + 1 >= minPathLength)
		{
			Vector3 imagePos = scene.camera.worldToRaster.tPoint(lightState.pos);
			if (scene.camera.checkRaster(imagePos.x , imagePos.y))
			{
				Color3 res = connectToCamera(lightState , lightState.pos , lightState.bsdf);

				film->addColor((int)imagePos.x , (int)imagePos.y , res);
			}
		}
	}

	// generating camera paths
	for (int index = 0; index < cameraPathNum; index++)
	{
		int pathIndex = index % (height * width);

		MMPathState cameraState;
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

			Vector3 hitPos = inter.p;

			BSDF bsdf(-ray.dir , inter , scene);
			if (!bsdf.isValid())
				break;

			cameraState.pos = hitPos;
			cameraState.bsdf = bsdf;

			if (inter.matId < 0)
			{
				AbstractLight *light = scene.lights[-inter.matId - 1];

				if (cameraState.pathLength >= minPathLength &&
					cameraState.specularPath)
				{
                    color = color + (cameraState.throughput |
						getLightRadiance(light , cameraState , 
						hitPos , ray.dir));
				}
				break;
			}

			if (cameraState.pathLength >= maxPathLength)
				break;

			// vertex merge
			GatherQuery query(*this , cameraState);
			if (!bsdf.isDelta)
			{
				lightTree->searchInRadius(0 , cameraState.pos , 
					radius , query);

				//fprintf(fp , "%d\n" , query.mergeNum);
			}

			if (!bsdf.isDelta)
			{
				if (cameraState.pathLength + 1 >= minPathLength)
				{
					color = color + (cameraState.throughput |
						getDirectIllumination(cameraState , hitPos , bsdf));
				}
			}

			color = color + (cameraState.throughput | query.contrib);

			Real pdf = 0;
			if (!sampleScattering(bsdf , hitPos , cameraState , &pdf , 1))
				break;

			Real weightFactor = connectFactor(pdf) / (connectFactor(pdf) +
				mergeFactor());

			cameraState.throughput = cameraState.throughput * weightFactor;
		}

		film->addColor((int)screenSample.x , (int)screenSample.y , color);
	}

    delete lightTree;
}

void MultipleMerge::generateLightSample(MMPathState& lightState)
{
	int lightNum = scene.lights.size();
	Real lightPickProb = 1.f / lightNum;

	int lightId = (int)(rng.randFloat() * lightNum);
	AbstractLight *light = scene.lights[lightId];

	Real emissionPdf , directPdf , cosAtLight;
	Color3 radiance = light->emit(scene.sceneSphere ,
		rng.randVector3() , rng.randVector3() , 
		lightState.origin , lightState.dir , emissionPdf ,
		&directPdf , &cosAtLight);

	emissionPdf *= lightPickProb;
	directPdf *= lightPickProb;

	lightState.throughput = Color3(1.f / emissionPdf);
	lightState.pathLength = 1;
	lightState.isFiniteLight = light->isFinite();

	lightState.dirContrib = radiance / emissionPdf;
	lightState.indirContrib = Color3(0.f);

	lightState.posAtOrigin = lightState.origin;
	lightState.dirAtOrigin = lightState.dir;
}

void MultipleMerge::generateInterSample(MMPathState& interState)
{
	/*
	int lightVertexNum = lightSubPaths.size();
	int vertexId = int(rng.randFloat() * lightVertexNum);
	MMPathState lightState = lightSubPaths[vertexId];
	Real vertexPickProb = 1.f / lightVertexNum;

	Real r = rng.randFloat() * radius;
	Real theta = rng.randFloat() * (2 * PI);

	Vector3 dx = lightState.bsdf.localFrame.binormal();
	Vector3 dy = lightState.bsdf.localFrame.tangent();
	*/

	int objNum = scene.objs.size();
	int objId; 
	for (;;)
	{
		objId = int(rng.randFloat() * objNum);
		int matId = scene.objs[objId]->getMatId();
		if (scene.materials[matId].specular.isBlack())
			break;
	}

	Vector3 normal;
	interState.posAtOrigin = scene.objs[objId]->samplePos(rng.randVector3() , normal);
	//interState.posAtOrigin = lightState.pos + dx * r * std::cos(theta) +
	//	dy * r * std::sin(theta);

	Real p = 0;
	Vector3 localDir = sampleUniformHemisphere(rng.randVector3() , &p);

	Frame localFrame;
	localFrame.buildFromZ(normal);
	interState.dirAtOrigin = localFrame.localToWorld(localDir);

	interState.throughput = Color3(1.f);
	interState.pathLength = 1;

	interState.origin = interState.posAtOrigin;
	interState.dir = interState.dirAtOrigin;

	interState.dirContrib = interState.indirContrib = Color3(0.f);
}

Color3 MultipleMerge::connectToCamera(MMPathState& lightState , 
	const Vector3& hitPos , BSDF& bsdf)
{
	Color3 res(0);

	Camera& camera = scene.camera;
	Vector3 dirToCamera = camera.pos - hitPos;

	if (cmp(-dirToCamera ^ camera.forward) <= 0)
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

	Real cosAtCamera = (-dirToCamera) ^ camera.forward;
	Real imagePointToCameraDist = camera.imagePlaneDist / cosAtCamera;
	Real imageToSolidAngleFactor = SQR(imagePointToCameraDist) / cosAtCamera;
	Real imageToSurfaceFactor = imageToSolidAngleFactor * std::abs(cosToCamera) / distEye2;

	Real cameraPdfArea = imageToSurfaceFactor /* * 1.f */; // pixel area is 1

	Real surfaceToImageFactor = 1.f / imageToSurfaceFactor;

	Color3 totContrib = lightState.dirContrib + lightState.indirContrib;

	res = (totContrib | bsdfFactor) /
		(lightPathNum * surfaceToImageFactor);
    
	if (res.isBlack())
		return res;

	if (scene.occluded(hitPos , dirToCamera , camera.pos))
		return Color3(0);

    Real glossyIndex = bsdf.glossyIndex;
	Real pdf = bsdfDirPdf;

    Real weightFactor = connectFactor(pdf) / 
		(connectFactor(pdf) + mergeFactor());
    
	return res * weightFactor;
}

bool MultipleMerge::sampleScattering(BSDF& bsdf , 
	const Vector3& hitPos , MMPathState& pathState , Real *_bsdfDirPdf ,
	bool isCameraPath)
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
	
	pathState.origin = hitPos;

	if (isCameraPath)
	{
		pathState.throughput = (pathState.throughput | bsdfFactor) *
			(std::abs(bsdf.cosWi()) / bsdfRevPdf);

		*_bsdfDirPdf = bsdfRevPdf;
	}
	else 
	{
		pathState.throughput = (pathState.throughput | bsdfFactor) *
			(cosWo / bsdfDirPdf);

		pathState.dirContrib = (pathState.dirContrib | bsdfFactor) *
			(cosWo / bsdfDirPdf);
		
		*_bsdfDirPdf = bsdfDirPdf;
	}

	if (sampledBSDFType & BSDF_SPECULAR)
	{
		pathState.specularPath &= 1;
		*_bsdfDirPdf = INF;
	}
	else
	{
		pathState.specularPath &= 0;
	}
	
	return 1;
}

Vector3 MultipleMerge::generateCameraSample(const int pathIndex , 
	MMPathState& cameraState)
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
	
    cameraState.throughput = Color3(1);

	cameraState.dirContrib = cameraState.indirContrib = Color3(0.f);

	cameraState.weight = 0.5f;
	cameraState.totWeight = 0.5f;

	return sample;
}

Color3 MultipleMerge::getLightRadiance(AbstractLight *light , 
	MMPathState& cameraState , const Vector3& hitPos , 
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

Color3 MultipleMerge::getDirectIllumination(MMPathState& cameraState , 
	const Vector3& hitPos , BSDF& bsdf)
{
    Color3 res(0);

	Real weightFactor = 0.f;

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

	Real pdf = directPdf;

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
			Real _weight = 1.f;

			Real lightPdf;
			if (!(sampledBSDFType & BSDF_SPECULAR))
			{
				illu = light->getRadiance(scene.sceneSphere , dirToLight , hitPos ,
					&lightPdf , &emissionPdf);

				if (cmp(lightPdf) == 0)
					return res;

				_weight = mis(directPdf) /
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
				res = res + tmp * _weight;
			}
		}
	}

    weightFactor = connectFactor(pdf) / (connectFactor(pdf) + mergeFactor());

	return res * weightFactor;

}

Color3 MultipleMerge::connectVertices(MMPathState& lightSubPath , 
	BSDF& cameraBsdf , const Vector3& hitPos , 
	MMPathState& cameraState)
{
	Vector3 dir = lightSubPath.pos - hitPos;
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
	Color3 lightBsdfFactor = lightSubPath.bsdf.f(scene , -dir , cosAtLight ,
		&lightBsdfDirPdf , &lightBsdfRevPdf);

	if (lightBsdfFactor.isBlack())
		return res;

	Real lightContProb = lightSubPath.bsdf.continueProb;
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

	return res;
}
