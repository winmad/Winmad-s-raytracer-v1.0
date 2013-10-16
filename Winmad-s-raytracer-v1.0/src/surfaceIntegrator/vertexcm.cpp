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

	for (int pathIndex = 0; pathIndex < pathNum; pathIndex++)
	{
		SubPathState lightState;
		generateLightSample(lightState);

		for (;; lightState.pathLength++)
		{
		}
	}
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

	}
	else
	{
	}
}

Real VertexCM::mis(Real pdf)
{
	return pdf;
}