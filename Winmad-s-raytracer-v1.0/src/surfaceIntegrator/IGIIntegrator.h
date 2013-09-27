#ifndef IGI_INTEGRATOR
#define IGI_INTEGRATOR

#include "../scene/scene.h"
#include "../parameters.h"
#include "surfaceIntegrator.h"
#include "../lightcuts/divisiveLightTree.h"

class IGIIntegrator : public SurfaceIntegrator
{
public:
	static const int NUM_DIRECT_LIGHT = 36;

	static const int NUM_INDIRECT_LIGHT = 100;

	Real threshold;

	int maxTracingDepth;

	int samplesOfHemisphere;

	int numDirectLight;

	int numIndirectLight;

	std::vector<VirtualLight> _directLights;
	std::vector<VirtualLight> _indirectLights;

	LightTree directLights;
	LightTree indirectLights;

	IGIIntegrator() {}

	void generateVirtualLights();

	void init(char *filename , Parameters& para);

	Color3 raytracing(const Ray& ray , int dep);

	void IGIIntegrator::render(char *filename);
};

#endif