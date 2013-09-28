#include "scene/scene.h"
#include "surfaceIntegrator/whitted.h"
#include "surfaceIntegrator/pathIntegrator.h"
#include "surfaceIntegrator/photonMap.h"
#include "surfaceIntegrator/IGIIntegrator.h"
#include "test/testTransform.h"
#include "parameters.h"
#include <opencv2/opencv.hpp>
#include "tinyxml/tinyxml.h"

Parameters para;
WhittedIntegrator whitted;
PathIntegrator pathIntegrator;
PhotonIntegrator photonIntegrator;
IGIIntegrator igiIntegrator;

int main(int argc , char* argv[])
{
	testAll();
	/*
    para.load_parameters("src/parameters.para");
	if (!strcmp(argv[3] , "-r"))
	{
		whitted.init(argv[1] , para);
		whitted.render(argv[2]);
	}
	else if (!strcmp(argv[3] , "-p"))
	{
		pathIntegrator.init(argv[1] , para);
		pathIntegrator.render(argv[2]);
	}
	else if (!strcmp(argv[3] , "-pm"))
	{
		photonIntegrator.init(argv[1] , para);
		photonIntegrator.buildPhotonMap(photonIntegrator.scene);
		//if (argc == 4)
			photonIntegrator.render(argv[2]);
	}
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