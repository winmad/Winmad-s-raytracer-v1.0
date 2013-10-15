#include "scene/scene.h"
#include "test/test.h"
#include "parameters.h"
#include "surfaceIntegrator/whitted.h"
#include "surfaceIntegrator/pathIntegrator.h"
#include "surfaceIntegrator/photonMap.h"
#include <opencv2/opencv.hpp>
#include "tinyxml/tinyxml.h"

Parameters para;
SurfaceIntegrator *integrator;
WhittedIntegrator whitted;
PathIntegrator pathIntegrator;
PhotonIntegrator photonIntegrator;

int main(int argc , char* argv[])
{
	para.load_parameters("src/parameters.para");
	if (!strcmp(argv[3] , "-r"))
	{
		whitted.init(argv[1] , para);
		whitted.render();
		whitted.outputImage(argv[2]);
	}
	else if (!strcmp(argv[3] , "-p"))
	{
		pathIntegrator.init(argv[1] , para);
		pathIntegrator.render();
		pathIntegrator.outputImage(argv[2]);
	}
	else if (!strcmp(argv[3] , "-pm"))
	{
		photonIntegrator.init(argv[1] , para);
		photonIntegrator.buildPhotonMap(photonIntegrator.scene);
		photonIntegrator.render();
		photonIntegrator.outputImage(argv[2]);
	}
	/*
	else if (!strcmp(argv[3] , "-igi"))
	{
		igiIntegrator.init(argv[1] , para);
		igiIntegrator.generateVirtualLights();
		igiIntegrator.render(argv[2]);
	}
	else
	{
		printf("error!\n");
	}
	*/
	return 0;
}
