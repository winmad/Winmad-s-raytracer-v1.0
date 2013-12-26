#ifndef SINGLE_SCATTERING_H
#define SINGLE_SCATTERING_H

#include "volumeIntegrator.h"

struct SSPathState
{
	Vector3 origin , pos;
	Vector3 dir;
	Color3 throughput;
	BSDF bsdf;

	int pathLength : 15;
	int specularVertexNum : 15;
	int specularPath : 1;
	int isFiniteLight : 1;
};

class SingleScattering : public VolumeIntegrator
{
public:
	int minPathLength , maxPathLength;
	int cameraPathNum;
	int iterations;
	Real stepSize;

	SingleScattering() {}

	void init(char *filename , Parameters& para);

	void runIteration(int iter);

	void render();

	void outputImage(char *filename);

	bool sampleScattering(BSDF& bsdf , const Vector3& hitPos , 
		SSPathState& pathState);

	Vector3 generateCameraSample(const int pathIndex , 
		SSPathState& cameraState);

	Color3 getLightRadiance(AbstractLight *light , 
		SSPathState& cameraState , const Vector3& hitPos , 
		const Vector3& rayDir);

	Color3 getDirectIllumination(SSPathState& cameraState , 
		const Vector3& hitPos , BSDF& bsdf);

	Color3 transmittance(const Ray& ray);

	Color3 getSingleScattering(const Ray& ray , Color3 *t);

	Real mis(Real pdf)
	{
		return pdf;
	}
};

#endif