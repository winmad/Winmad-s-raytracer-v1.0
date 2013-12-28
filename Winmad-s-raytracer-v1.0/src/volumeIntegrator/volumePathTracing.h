#ifndef VOLUME_PATH_TRACING_H
#define VOLUME_PATH_TRACING_H

#include "volumeIntegrator.h"

struct VptPathState
{
	Vector3 origin , pos;
	Vector3 dir;
	Color3 throughput;
	BSDF bsdf;
	Volume *vr;

	int pathLength : 15;
	int specularVertexNum : 15;
	int specularPath : 1;
	int isFiniteLight : 1;
};

class VolumePathTracing : public VolumeIntegrator
{
public:
	int minPathLength , maxPathLength;
	int cameraPathNum;
	int iterations;
	Real stepSize;

	VolumePathTracing() {}

	void init(char *filename , Parameters& para);

	void runIteration(int iter);

	void render();

	void outputImage(char *filename);

	bool sampleScattering(BSDF& bsdf , const Vector3& hitPos , 
		VptPathState& pathState , int& inOutFlag);

	Vector3 generateCameraSample(const int pathIndex , 
		VptPathState& cameraState);

	Color3 getLightRadiance(AbstractLight *light , 
		VptPathState& cameraState , const Vector3& hitPos , 
		const Vector3& rayDir);

	Color3 getDirectIllumination(VptPathState& cameraState , 
		const Vector3& hitPos , BSDF& bsdf);

	Color3 transmittance(Volume *vr , const Ray& ray);

	Real getMediaScattering(Volume* vr , Ray& ray , Real& t0 , 
		Real& t1 , Real& marchLen , Vector3& pos , Vector3& dir , Color3& tr);

	Color3 handleSurface(VptPathState& cameraState , Ray& ray ,
		Intersection& inter , bool& contFlag , int& inOutFlag);

	Color3 handleVolume(VptPathState& cameraState , 
		Ray& ray , Real& t0 , Real& t1 , Color3& tr , Vector3& pos , 
		Vector3& dir , Real& pdf); 

	Real mis(Real pdf)
	{
		return pdf;
	}
};

#endif