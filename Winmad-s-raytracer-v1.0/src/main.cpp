#include "scene/scene.h"
#include "test/test.h"
#include "parameters.h"
#include "surfaceIntegrator/whitted.h"
#include "surfaceIntegrator/pathIntegrator.h"
#include "surfaceIntegrator/photonMap.h"
#include "surfaceIntegrator/vertexcm.h"
#include "surfaceIntegrator/pathReusing.h"
#include "surfaceIntegrator/bidirPathTracing.h"
#include "surfaceIntegrator/multipleMerge.h"
#include <opencv2/opencv.hpp>
#include "tinyxml/tinyxml.h"
#include <ctime>

Parameters para;
SurfaceIntegrator *integrator;
WhittedIntegrator whitted;
PathIntegrator pathIntegrator;
PhotonIntegrator photonIntegrator;
VertexCM vertexcmIntegrator;
PathReusing pathReusing;
BidirPathTracing bidirPathTracing;
MultipleMerge multipleMerge;

int main(int argc , char* argv[])
{
	clock_t start , end;
	para.load_parameters("src/parameters.para");
	start = clock();
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
	else if (!strcmp(argv[3] , "-vcm"))
	{
		vertexcmIntegrator.init(argv[1] , para);
		vertexcmIntegrator.render();
		vertexcmIntegrator.outputImage(argv[2]);
	}
	else if (!strcmp(argv[3] , "-pr"))
	{
		pathReusing.init(argv[1] , para);
		pathReusing.render();
		pathReusing.outputImage(argv[2]);
	}
    else if (!strcmp(argv[3] , "-bpt"))
    {
        bidirPathTracing.init(argv[1] , para);
        bidirPathTracing.render();
        bidirPathTracing.outputImage(argv[2]);
    }
	else if (!strcmp(argv[3] , "-mm"))
	{
		multipleMerge.init(argv[1] , para);
		multipleMerge.render();
		multipleMerge.outputImage(argv[2]);
	}
	else
	{
		printf("error!\n");
	}
	end = clock();
	FILE *fp = fopen("time.txt" , "w");
	fprintf(fp , "time = %d\n" , end - start);
	return 0;
}
