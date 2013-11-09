#ifndef BIDIR_PATH_TRACING_H
#define BIDIR_PATH_TRACING_H

#include "surfaceIntegrator.h"

struct BidirPathState
{
	Vector3 origin , pos;
	Vector3 dir;
	Color3 throughput;
	BSDF bsdf;
	Real pdf;

	int pathLength : 30;
	int specularPath : 1;
	int isFiniteLight : 1;
};

class BidirPathTracing : public SurfaceIntegrator
{
public:
	int minPathLength , maxPathLength;
	int lightPathNum , cameraPathNum;
	int iterations;

	std::vector<BidirPathState> lightStates;
	std::vector<BidirPathState> cameraStates;

    std::vector<int> lightStateIndex;
    std::vector<int> cameraStateIndex;
    
	BidirPathTracing() {}

	void init(char *filename , Parameters& para);

	void runIteration(int iter);

	void render();

	void outputImage(char *filename);

	void generateLightSample(BidirPathState& lightState);

	Color3 connectToCamera(BidirPathState& lightState , const Vector3& hitPos , BSDF& bsdf , Real *cameraDirPdf = NULL , Real *cameraRevPdf = NULL);

	bool sampleScattering(BSDF& bsdf , const Vector3& hitPos , 
		BidirPathState& pathState);

	Vector3 generateCameraSample(const int pathIndex , 
		BidirPathState& cameraState);

	Color3 getLightRadiance(AbstractLight *light , BidirPathState& cameraState , const Vector3& hitPos , const Vector3& rayDir , Real *lightDirPdfArea = NULL , Real *lightRevPdfArea = NULL);

	Color3 getDirectIllumination(BidirPathState& cameraState , const Vector3& hitPos , BSDF& bsdf , Real *lightDirPdf = NULL , Real *lightRevPdf = NULL , Real *cameraDirPdf = NULL , Real *cameraRevPdf = NULL);

	Color3 connectVertices(BidirPathState& lightState , BSDF& bsdf , const Vector3& hitPos , BidirPathState& cameraState , Real *lightDirPdf = NULL , Real *lightRevPdf = NULL , Real *cameraDirPdf = NULL , Real *cameraRevPdf = NULL);

	Real mis(Real pdf);
};

#endif
