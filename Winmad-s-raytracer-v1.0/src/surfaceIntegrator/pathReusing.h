#ifndef PATH_REUSING_H
#define PATH_REUSING_H

#include "surfaceIntegrator.h"

struct PathState
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

class PathReusing : public SurfaceIntegrator
{
public:
	int minPathLength , maxPathLength;
	int lightPathNum , cameraPathNum;
	int iterations;

	std::vector<PathState> lightStates;
	std::vector<PathState> cameraStates;

	std::vector<int> lightStateIndex;
	std::vector<int> cameraStateIndex;

	PathReusing() {}

	void init(char *filename , Parameters& para);

	void runIteration(int iter);

	void render();

	void outputImage(char *filename);

	void generateLightSample(PathState& lightState);

	bool sampleScattering(BSDF& bsdf , const Vector3& hitPos , 
		PathState& pathState);

	Vector3 generateCameraSample(const int pathIndex , 
		PathState& cameraState);

	Color3 getLightRadiance(AbstractLight *light , PathState& cameraState ,
		const Vector3& hitPos , const Vector3& rayDir);

	Color3 getDirectIllumination(PathState& cameraState , const Vector3& hitPos ,
		BSDF& bsdf);

	Color3 connectVertices(PathState& lightState , BSDF& bsdf , 
		const Vector3& hitPos , PathState& cameraState);

	Real mis(Real pdf);
};

#endif