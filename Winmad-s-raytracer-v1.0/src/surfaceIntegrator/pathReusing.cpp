#include "pathReusing.h"

static FILE *fp = fopen("debug_pr.txt" , "w");

void PathReusing::init(char *filename , Parameters& para)
{
	minPathLength = 0;
	maxPathLength = 10;
	iterations = 30;

	samplesPerPixel = para.SAMPLES_PER_PIXEL;

	scene.init(filename , para);

	baseRadius = 0.003f * scene.sceneSphere.sceneRadius;
	radiusAlpha = 0.75f;

	height = para.HEIGHT; width = para.WIDTH;

	film = new ImageFilm(height , width);
}

void PathReusing::render()
{
	for (int iter = 0; iter < iterations; iter++)
		runIteration(iter);
}

void PathReusing::outputImage(char *filename)
{
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < i; j++)
		{
			Color3 tmp = film->color[i][j];
			film->color[i][j] = film->color[j][i];
			film->color[j][i] = tmp;

			tmp = film->color[i][j];
			fprintf(fp , "c(%d,%d)=(%.3f,%.3f,%.3f)\n" , i , j ,
				tmp.r , tmp.g , tmp.b);
		}
	}
	film->outputImage(filename , 1.f / iterations , 2.2);
}

void PathReusing::runIteration(int iter)
{
	lightPathNum = height * width;
	cameraPathNum = lightPathNum;

	Real radius = baseRadius;
	radius /= std::pow((Real)(iter + 1) , 0.5f * (1.f - radiusAlpha));
	radius = std::max(radius , EPS);
	Real radiusSqr = SQR(radius);

	lightStateIndex.resize(lightPathNum);
	memset(&lightStateIndex[0] , 0 , lightStateIndex.size() * sizeof(int));

	lightStates.reserve(lightPathNum);
	lightStates.clear();

	cameraSubPaths[0].reserve(cameraPathNum);
	cameraSubPaths[0].clear();
	cameraSubPaths[1].reserve(cameraPathNum);
	cameraSubPaths[1].clear();

	// generating light paths
	for (int pathIndex = 0; pathIndex < lightPathNum; pathIndex++)
	{
		PathState lightState;
		generateLightSample(lightState);

		for (;; lightState.pathLength++)
		{
			Ray ray(lightState.pos + lightState.dir * EPS ,
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
			lightState.pdf *= std::abs(bsdf.cosWi()) / SQR(inter.t);

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
						film->addColor((int)imagePos.x , (int)imagePos.y , res);
					}
				}
			}

			if (lightState.pathLength + 2 > maxPathLength)
				break;

			if (!sampleScattering(bsdf , hitPos , lightState))
				break;
		}

		lightStateIndex[pathIndex] = (int)lightStateIndex.size();
	}

	// generating camera paths
	for (int pathIndex = 0; pathIndex < cameraPathNum; pathIndex++)
	{
		PathState cameraState , oldCameraState;
		Vector3 screenSample = generateCameraSample(pathIndex , cameraState);

		oldCameraState = cameraState;
		oldCameraState.pdf = 1;
		oldCameraState.throughput = Color3(1.f);

		Color3 color(0);

		Vector3 dirAtOrigin = cameraState.dir;
		bool isNewSubPath = 0;
		bool isStart = 1;

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
						color = color + (cameraState.throughput |
							getLightRadiance(scene.background ,
							cameraState , Vector3(0) , ray.dir));
					}
				}
				break;
			}

			Vector3 hitPos = inter.p;

			BSDF bsdf(-ray.dir , inter , scene);
			if (!bsdf.isValid())
				break;

			cameraState.pdf *= std::abs(bsdf.cosWi()) / SQR(inter.t);

			if (inter.matId < 0)
			{
				AbstractLight *light = scene.lights[-inter.matId - 1];

				if (cameraState.pathLength >= minPathLength)
				{
					color = color + (cameraState.throughput |
						getLightRadiance(light , cameraState , 
						hitPos , ray.dir));
				}
				break;
			}

			if (cameraState.pathLength >= maxPathLength)
				break;

			if (cameraState.pathLength == 1 && bsdf.isDelta)
			{
				int a = 0;
			}

			// store path state
			if (!bsdf.isDelta)
			{
				SubPath subPath(oldCameraState , cameraState);
				subPath.nextPos = inter.p;
				subPath.bsdf = BSDF(-ray.dir , inter , scene);
				subPath.wo = dirAtOrigin;
				if (isStart)
				{
					isStart = 0;
					subPath.rasterX = (int)screenSample.x;
					subPath.rasterY = (int)screenSample.y;
				}

				cameraSubPaths[0].push_back(subPath);
				cameraSubPaths[1].push_back(subPath);
				
				oldCameraState = cameraState;
				oldCameraState.origin = inter.p;
				
				isNewSubPath = 1;
				
				
// 				SubPath subPath;
// 				subPath.pos = cameraState.origin;
// 				subPath.nextPos = inter.p;
// 				subPath.throughput = cameraState.throughput;
// 				subPath.pdf = cameraState.pdf;
// 				subPath.bsdf = bsdf;
// 				subPath.rasterX = (int)screenSample.x;
// 				subPath.rasterY = (int)screenSample.y;
// 				cameraSubPaths.push_back(subPath);
				
			}

			// vertex connection: connect to light source
			if (!bsdf.isDelta)
			{
				if (cameraState.pathLength + 1 >= minPathLength)
				{
					color = color + (cameraState.throughput |
						getDirectIllumination(cameraState , hitPos , bsdf));
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
					PathState& lightState = lightStates[i];

					if (lightState.pathLength + 1 + 
						cameraState.pathLength < minPathLength)
						continue;

					if (lightState.pathLength + 1 +
						cameraState.pathLength > maxPathLength)
						break;

					Color3 tmp = connectVertices(lightState ,
						bsdf , hitPos , cameraState);

					color = color + (cameraState.throughput |
						lightState.throughput | tmp);
				}
			}

			if (!sampleScattering(bsdf , hitPos , cameraState))
				break;

			if (isNewSubPath)
			{
				isNewSubPath = 0;
				dirAtOrigin = cameraState.dir;
			}
		}

		film->addColor((int)screenSample.x , (int)screenSample.y , color);
	}

	// merge
	lightTree = new KdTree<PathState>(lightStates);

	Real kernel = 1.f / (PI * radiusSqr * lightStates.size());

	for (int i = 0; i < cameraSubPaths[0].size(); i++)
	{
		RangeQuery query(*this , cameraSubPaths[0][i].nextPos , 
			cameraSubPaths[0][i].bsdf , cameraSubPaths[0][i]);
		
		lightTree->searchInRadius(0 , cameraSubPaths[0][i].nextPos , 
			radius , query);

		//fprintf(fp , "%d\n" , query.mergeNum);

		Color3 color = (cameraSubPaths[0][i].throughput | query.contrib) *
			kernel;

		cameraSubPaths[0][i].contrib = cameraSubPaths[0][i].contrib +
			color;

// 		if (cameraSubPaths[0][i].isStart())
// 		{
// 			film->addColor(cameraSubPaths[0][i].rasterX , 
// 				cameraSubPaths[0][i].rasterY , cameraSubPaths[0][i].contrib);
// 		}
	}

	delete lightTree;

	kernel = 1.f / (PI * radiusSqr * cameraSubPaths[0].size());

	int now = 0;
	for (int mergeIter = 0; mergeIter < maxPathLength; mergeIter++)
	{
		now ^= 1;
		pathTree = new KdTree<SubPath>(cameraSubPaths[now ^ 1]);

		for (int i = 0; i < cameraSubPaths[now].size(); i++)
		{
			MergeQuery query(*this , cameraSubPaths[now][i].nextPos ,
				cameraSubPaths[now][i]);

			pathTree->searchInRadius(0 , query.pathEnd , radius , query);

			//fprintf(fp , "%d\n" , query.mergeNum);

			Color3 color = (cameraSubPaths[now][i].throughput | query.contrib) *
				kernel;

			cameraSubPaths[now][i].contrib = cameraSubPaths[now][i].contrib +
				color;
		}
		
		delete pathTree;
	}

	for (int i = 0; i < cameraSubPaths[now].size(); i++)
	{
		if (cameraSubPaths[now][i].isStart())
		{
			film->addColor(cameraSubPaths[now][i].rasterX , 
				cameraSubPaths[now][i].rasterY , cameraSubPaths[now][i].contrib);
		}
	}
}

void PathReusing::generateLightSample(PathState& lightState)
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

	lightState.pdf = emissionPdf;

	lightState.throughput = lightState.throughput;
}

Color3 PathReusing::connectToCamera(PathState& lightState , 
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
	Real imageToSurfaceFactor = imageToSolidAngleFactor * std::abs(cosAtCamera) / distEye2;

	Real cameraPdfArea = imageToSurfaceFactor /* * 1.f */; // pixel area is 1

	Real surfaceToImageFactor = 1.f / imageToSurfaceFactor;

	//res = (lightState.throughput | bsdfFactor) /
	//	(cameraPathNum * surfaceToImageFactor);

	res = (lightState.throughput | bsdfFactor) /
		(cameraPdfArea * cameraPathNum);

	if (res.isBlack())
		return res;

	if (scene.occluded(hitPos , dirToCamera , camera.pos))
		return Color3(0);

	return res;
}

bool PathReusing::sampleScattering(BSDF& bsdf , 
	const Vector3& hitPos , PathState& pathState)
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
		pathState.pdf *= 1.f;
	}
	else
	{
		pathState.pdf *= bsdfDirPdf;
		pathState.specularPath &= 0;
	}

	pathState.origin = hitPos;
	pathState.throughput = (pathState.throughput | bsdfFactor) *
		(cosWo / bsdfDirPdf);

	return 1;
}

Vector3 PathReusing::generateCameraSample(const int pathIndex , 
	PathState& cameraState)
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
	cameraState.pdf = cameraPdf / cameraPathNum;

	cameraState.throughput = Color3(1) / cameraState.pdf;
	
	return sample;
}

Color3 PathReusing::getLightRadiance(AbstractLight *light , 
	PathState& cameraState , const Vector3& hitPos , 
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
	
	Real pdf = directPdfArea;

	return radiance / pdf;
}

Color3 PathReusing::getDirectIllumination(PathState& cameraState , 
	const Vector3& hitPos , BSDF& bsdf)
{
	Color3 res(0);

	int lightCount = scene.lights.size();
	Real lightPickProb = 1.f / lightCount;

	int lightId = (int)(rng.randFloat() * lightCount);
	AbstractLight *light = scene.lights[lightId];

	Vector3 dirToLight;
	Real dist , directPdf , emissionPdf , cosAtLight;

	Color3 illu = light->illuminance(scene.sceneSphere ,
		hitPos , rng.randVector3() , dirToLight , dist ,
		directPdf , &emissionPdf , &cosAtLight);

	if (illu.isBlack())
		return res;

	Real bsdfDirPdf , bsdfRevPdf , cosToLight;

	Color3 bsdfFactor = bsdf.f(scene , dirToLight ,
		cosToLight , &bsdfDirPdf , &bsdfRevPdf);

	if (bsdfFactor.isBlack())
		return res;

	Real contProb = bsdf.continueProb;

	bsdfDirPdf *= light->isDelta() ? 0.f : contProb;
	bsdfRevPdf *= contProb;

	res = (illu | bsdfFactor) / (directPdf * lightPickProb);

	if (res.isBlack() || scene.occluded(hitPos , dirToLight ,
		hitPos + dirToLight * dist))
		return Color3(0);

	return res;
}

Color3 PathReusing::connectVertices(PathState& lightState , 
	BSDF& cameraBsdf , const Vector3& hitPos , 
	PathState& cameraState)
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

	return res;
}

Real PathReusing::mis(Real pdf)
{
	return pdf * pdf;
}