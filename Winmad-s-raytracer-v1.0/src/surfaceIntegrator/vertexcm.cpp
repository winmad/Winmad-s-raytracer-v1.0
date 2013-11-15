#include "vertexcm.h"

void VertexCM::init(char *filename , Parameters& para)
{
	minPathLength = 0;
	maxPathLength = 10;
	iterations = 1;

	samplesPerPixel = para.SAMPLES_PER_PIXEL;
	
	scene.init(filename , para);

	baseRadius = 0.003f * scene.sceneSphere.sceneRadius;
	radiusAlpha = 0.75f;

	height = para.HEIGHT; width = para.WIDTH;

	film = new ImageFilm(height , width);

	tree = NULL;
}

void VertexCM::render()
{
	for (int iter = 0; iter < iterations; iter++)
		runIteration(iter);
}

//static FILE *fp = fopen("debug_vcm.txt" , "w");

void VertexCM::outputImage(char *filename)
{
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < i; j++)
		{
			Color3 tmp = film->color[i][j];
			film->color[i][j] = film->color[j][i];
			film->color[j][i] = tmp;
		}
	}
	film->outputImage(filename , 1.f / iterations , 2.2);
}

void VertexCM::runIteration(int iter)
{
	int pathNum = height * width;
	screenPixelNum = (Real)(height * width);
	lightSubPathNum = (Real)(height * width);

	Real radius = baseRadius;
	radius /= std::pow((Real)(iter + 1) , 0.5f * (1.f - radiusAlpha));
	radius = std::max(radius , EPS);
	Real radiusSqr = SQR(radius);

	vmNormalization = 1.f / (radiusSqr * PI * lightSubPathNum);

	Real etaVCM = (PI * radiusSqr) * lightSubPathNum;
	misVmWeightFactor = mis(etaVCM);
	misVcWeightFactor = mis(1.f / etaVCM);

	pathEnds.resize(pathNum);
	memset(&pathEnds[0] , 0 , pathEnds.size() * sizeof(int));

	lightVertices.reserve(pathNum);
	lightVertices.clear();

	////////////////////////////////////////////////////////////
	// generate light paths
	////////////////////////////////////////////////////////////
	for (int pathIndex = 0; pathIndex < pathNum; pathIndex++)
	{
		SubPathState lightState;
		generateLightSample(lightState);

		for (;; lightState.pathLength++)
		{
			Ray ray(lightState.pathOrigin + lightState.dir * EPS ,
				lightState.dir);
			Intersection inter;
			if (scene.intersect(ray , inter) == NULL)
				break;

			Vector3 hitPos = inter.p;

			BSDF bsdf(-ray.dir , inter , scene);
			if (!bsdf.isValid())
				break;

			if (lightState.pathLength > 1 || lightState.isFiniteLight == 1)
				lightState.dVCM *= mis(SQR(inter.t));

			lightState.dVCM /= mis(std::abs(bsdf.cosWi()));
			lightState.dVC /= mis(std::abs(bsdf.cosWi()));
			lightState.dVM /= mis(std::abs(bsdf.cosWi()));

			// store lightVertex
			if (!bsdf.isDelta)
			{
				PathVertex lightVertex;
				lightVertex.pos = hitPos;
				lightVertex.throughput = lightState.throughput;
				lightVertex.pathLength = lightState.pathLength;
				lightVertex.bsdf = bsdf;
				lightVertex.dVCM = lightState.dVCM;
				lightVertex.dVC = lightState.dVC;
				lightVertex.dVM = lightState.dVM;

				lightVertices.push_back(lightVertex);
			}

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

		pathEnds[pathIndex] = (int)lightVertices.size();
	}

	////////////////////////////////////////////////////////////
	// build vertex kd-tree
	////////////////////////////////////////////////////////////
	tree = new KdTree<PathVertex>(lightVertices);

	////////////////////////////////////////////////////////////
	// generate camera paths
	////////////////////////////////////////////////////////////
	for (int pathIndex = 0; pathIndex < pathNum; pathIndex++)
	{
		SubPathState cameraState;
		Vector3 screenSample = generateCameraSample(pathIndex , cameraState);
		Color3 color(0);

		for (;; cameraState.pathLength++)
		{
			Ray ray(cameraState.pathOrigin + cameraState.dir * EPS ,
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

			cameraState.dVCM *= mis(SQR(inter.t));
			cameraState.dVCM /= mis(std::abs(bsdf.cosWi()));
			cameraState.dVC /= mis(std::abs(bsdf.cosWi()));
			cameraState.dVM /= mis(std::abs(bsdf.cosWi()));

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
					st = pathEnds[pathIndex - 1];
				ed = pathEnds[pathIndex];
				
				for (int i = st; i < ed; i++)
				{
					PathVertex& lightVertex = lightVertices[i];

					if (lightVertex.pathLength + 1 + 
						cameraState.pathLength < minPathLength)
						continue;

					if (lightVertex.pathLength + 1 +
						cameraState.pathLength > maxPathLength)
						break;

					Color3 tmp = connectVertices(lightVertex ,
						bsdf , hitPos , cameraState);

					color = color + (cameraState.throughput |
						lightVertex.throughput | tmp);
				}
			}

			// vertex merging
			if (!bsdf.isDelta)
			{
				RangeQuery query(*this , hitPos , bsdf , cameraState);

				tree->searchInRadius(0 , hitPos , radius , query);

				//fprintf(fp , "%d\n" , query.mergeNum);

				color = color + (cameraState.throughput | query.contrib) *
					vmNormalization;
			}

			if (!sampleScattering(bsdf , hitPos , cameraState))
				break;
		}

		film->addColor((int)screenSample.x , (int)screenSample.y , color);
	}
	delete tree;
}

void VertexCM::generateLightSample(SubPathState& lightState)
{
	int lightNum = scene.lights.size();
	Real lightPickProb = 1.f / lightNum;

	int lightId = (int)(rng.randFloat() * lightNum);
	AbstractLight *light = scene.lights[lightId];

	Real emissionPdf , directPdf , cosAtLight;
	lightState.throughput = light->emit(scene.sceneSphere ,
		rng.randVector3() , rng.randVector3() , 
		lightState.pathOrigin , lightState.dir , emissionPdf ,
		&directPdf , &cosAtLight);

	emissionPdf *= lightPickProb;
	directPdf *= lightPickProb;

	lightState.throughput = lightState.throughput / emissionPdf;
	lightState.pathLength = 1;
	lightState.isFiniteLight = light->isFinite();

	lightState.dVCM = mis(directPdf / emissionPdf);

	if (!light->isDelta())
	{
		Real cosLight = light->isFinite() ? cosAtLight : 1.f;
		lightState.dVC = mis(cosLight / emissionPdf);
	}
	else
	{
		lightState.dVC = 0.f;
	}

	lightState.dVM = lightState.dVC * misVcWeightFactor;
}

Color3 VertexCM::connectToCamera(const SubPathState& lightState , 
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

	Real weightLight = mis(cameraPdfArea / lightSubPathNum) * 
		(misVmWeightFactor + lightState.dVCM + lightState.dVC * mis(bsdfRevPdf));

	Real weightCamera = 0.f;

	Real misWeight = 1.f / (weightLight + 1.f);

	Real surfaceToImageFactor = 1.f / imageToSurfaceFactor;

	res = (lightState.throughput | bsdfFactor) * misWeight /
		(lightSubPathNum * surfaceToImageFactor);

	if (res.isBlack())
		return res;

	if (scene.occluded(hitPos , dirToCamera , camera.pos))
		return Color3(0);

	fprintf(fp , "s=%d,t=%d,w=%.6f\n" , lightState.pathLength , 0 , misWeight);
	return res;
}

bool VertexCM::sampleScattering(BSDF& bsdf , 
	const Vector3& hitPos , SubPathState& pathState)
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
		pathState.dVCM = 0.f;
		assert(bsdfDirPdf == bsdfRevPdf);
		pathState.dVC *= mis(cosWo);
		pathState.dVM *= mis(cosWo);
	}
	else
	{
		pathState.dVC = mis(cosWo / bsdfDirPdf) *
			(pathState.dVC * mis(bsdfRevPdf) + pathState.dVCM +
			misVmWeightFactor);
		pathState.dVM = mis(cosWo / bsdfDirPdf) *
			(pathState.dVM * mis(bsdfRevPdf) + 
			pathState.dVCM * misVcWeightFactor + 1.f);
		pathState.dVCM = mis(1.f / bsdfDirPdf);
		pathState.specularPath &= 0;
	}

	pathState.pathOrigin = hitPos;
	pathState.throughput = (pathState.throughput | bsdfFactor) *
		(cosWo / bsdfDirPdf);

	return 1;
}

Vector3 VertexCM::generateCameraSample(const int pathIndex , 
	SubPathState& cameraState)
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

	cameraState.pathOrigin = ray.origin;
	cameraState.dir = ray.dir;
	cameraState.throughput = Color3(1);

	cameraState.pathLength = 1;
	cameraState.specularPath = 1;

	cameraState.dVCM = mis(lightSubPathNum / cameraPdf);
	cameraState.dVC = 0;
	cameraState.dVM = 0;

	return sample;
}

Color3 VertexCM::getLightRadiance(AbstractLight *light , 
	SubPathState& cameraState , const Vector3& hitPos , 
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

	Real weightCamera = mis(directPdfArea) * cameraState.dVCM +
		mis(emissionPdf) * cameraState.dVC;

	Real weightLight = 0.f;

	Real misWeight = 1.f / (1.f + weightCamera);

	//fprintf(fp , "s=%d,t=%d,w=%.6f\n" , 0 , cameraState.pathLength , misWeight);

	return radiance * misWeight;
}

Color3 VertexCM::getDirectIllumination(SubPathState& cameraState , 
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

	Real weightLight = mis(bsdfDirPdf / 
		(lightPickProb * directPdf));

	Real weightCamera = mis(emissionPdf * cosToLight /
		(directPdf * cosAtLight)) * (misVmWeightFactor +
		cameraState.dVCM + cameraState.dVC * mis(bsdfRevPdf));

	Real misWeight = 1.f / (weightLight + 1.f + weightCamera);

	res = (illu | bsdfFactor) * (misWeight * cosToLight / 
		(lightPickProb * directPdf));

	if (res.isBlack() || scene.occluded(hitPos , dirToLight ,
		hitPos + dirToLight * dist))
		return Color3(0);

	//fprintf(fp , "s=%d,t=%d,w=%.6f\n" , 1 , cameraState.pathLength , misWeight);

	return res;
}

Color3 VertexCM::connectVertices(PathVertex& lightVertex , 
	BSDF& cameraBsdf , const Vector3& cameraHitPos , 
	SubPathState& cameraState)
{
	Vector3 dir = lightVertex.pos - cameraHitPos;
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
	Color3 lightBsdfFactor = lightVertex.bsdf.f(scene , -dir , cosAtLight ,
		&lightBsdfDirPdf , &lightBsdfRevPdf);

	if (lightBsdfFactor.isBlack())
		return res;

	Real lightContProb = lightVertex.bsdf.continueProb;
	lightBsdfDirPdf *= lightContProb;
	lightBsdfRevPdf *= lightContProb;

	Real geometryTerm = cosAtLight * cosAtCamera / dist2;
	if (cmp(geometryTerm) < 0)
		return res;

	Real cameraBsdfDirPdfArea = pdfWtoA(cameraBsdfDirPdf , dist , cosAtLight);
	Real lightBsdfDirPdfArea = pdfWtoA(lightBsdfDirPdf , dist , cosAtCamera);

	Real weightLight = mis(cameraBsdfDirPdfArea) *
		(misVmWeightFactor + lightVertex.dVCM + 
		lightVertex.dVC * mis(lightBsdfRevPdf));

	Real weightCamera = mis(lightBsdfDirPdfArea) *
		(misVmWeightFactor + cameraState.dVCM +
		cameraState.dVC * mis(cameraBsdfRevPdf));

	Real misWeight = 1.f / (weightLight + 1.f + weightCamera);

	res = (cameraBsdfFactor | lightBsdfFactor) * misWeight * geometryTerm;

	if (res.isBlack() || scene.occluded(cameraHitPos , dir , 
		cameraHitPos + dir * dist))
		return Color3(0);

	//fprintf(fp , "s=%d,t=%d,w=%.6f\n" , lightVertex.pathLength , 
	//	cameraState.pathLength , misWeight);

	return res;
}

Real VertexCM::mis(Real pdf)
{
	return pdf;
}