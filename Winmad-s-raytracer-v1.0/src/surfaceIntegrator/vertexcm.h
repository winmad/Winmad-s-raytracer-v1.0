#ifndef VERTEX_CM_H
#define VERTEX_CM_H

#include "surfaceIntegrator.h"
#include "../math/color.h"
#include "../math/vector.h"
#include "../material/bsdf.h"
#include <vector>

static FILE *fp = fopen("debug_vcm.txt" , "w");

struct PathVertex
{
	Vector3 pos;
	Color3 throughput;
	int pathLength;

	std::string pathHistory;

	BSDF bsdf;

	Real dVCM , dVC , dVM;
};

struct SubPathState
{
	Vector3 pathOrigin;
	Vector3 dir;
	Color3 throughput;
	int pathLength : 30;
	int isFiniteLight : 1;
	int specularPath : 1;

	std::string pathHistory;

	Real dVCM , dVC , dVM;
};

class VertexCM : public SurfaceIntegrator
{
public:
	class RangeQuery
	{
	public:
		VertexCM& vertexCM;
		Vector3& cameraPos;
		BSDF& cameraBsdf;
		SubPathState& cameraState;
		Color3 contrib;
		int mergeNum;

		RangeQuery(VertexCM& vertexCM , 
			Vector3& cameraPos , BSDF& cameraBsdf ,
			SubPathState& cameraState)
			: vertexCM(vertexCM) , cameraPos(cameraPos) ,
			cameraBsdf(cameraBsdf) , cameraState(cameraState) ,
			contrib(0) , mergeNum(0) {}

		void process(PathVertex& lightVertex)
		{
			if (lightVertex.pathLength + cameraState.pathLength > vertexCM.maxPathLength ||
				lightVertex.pathLength + cameraState.pathLength < vertexCM.minPathLength)
				return;

			Vector3 lightDir = lightVertex.bsdf.wiWorld();

			Real cosCamera , cameraBsdfDirPdf , cameraBsdfRevPdf;

			Color3 cameraBsdfFactor = cameraBsdf.f(vertexCM.scene , 
				lightDir , cosCamera , &cameraBsdfDirPdf , &cameraBsdfRevPdf);

			if (cameraBsdfFactor.isBlack())
				return;

			mergeNum++;

			cameraBsdfDirPdf *= cameraBsdf.continueProb;
			cameraBsdfRevPdf *= lightVertex.bsdf.continueProb;

			Real weightLight = lightVertex.dVCM * vertexCM.misVcWeightFactor +
				lightVertex.dVM * vertexCM.mis(cameraBsdfDirPdf);

			Real weightCamera = cameraState.dVCM * vertexCM.misVcWeightFactor +
				cameraState.dVM * vertexCM.mis(cameraBsdfRevPdf);

			Real misWeight = 1.f / (weightLight + 1.f + weightCamera);

			Color3 tmp = (cameraBsdfFactor | lightVertex.throughput);
			/*
			fprintf(fp , "vm s=%d,t=%d,LightPath=%s,CameraPath=%s,w=%.6f\nContrib=(%.4f,%.4f,%.4f)\n" , lightVertex.pathLength , 
				cameraState.pathLength , lightVertex.pathHistory.c_str() ,
				cameraState.pathHistory.c_str() , misWeight , tmp.r ,
				tmp.g , tmp.b);
			*/
			contrib = contrib + tmp * misWeight;
		}
	};

	int minPathLength , maxPathLength;
	int iterations;
	Real baseRadius;         // initial merging radius
	Real radiusAlpha;        // radius reduction per iteration
	Real misVmWeightFactor;
	Real misVcWeightFactor;
	Real screenPixelNum;
	Real lightSubPathNum;
	Real vmNormalization;    // 1 / (PI * radius ^ 2 * lightSubPathNum)

	std::vector<PathVertex> lightVertices;
	std::vector<int> pathEnds;

	KdTree<PathVertex> *tree;

	VertexCM() {}

	void init(char *filename , Parameters& para);

	void runIteration(int iter);

	void render();

	void outputImage(char *filename);

	/***********************
	 *	light path tracing */
	void generateLightSample(SubPathState& lightState);

	Color3 connectToCamera(const SubPathState& lightState ,
		const Vector3& hitPos , BSDF& bsdf);

	// sample both light path and camera path
	bool sampleScattering(BSDF& bsdf , const Vector3& hitPos , 
		SubPathState& pathState);
	/***********************/

	/***********************
	 *	camera path tracing */
	Vector3 generateCameraSample(const int pathIndex , 
		SubPathState& cameraState);

	Color3 getLightRadiance(AbstractLight *light , 
		SubPathState& cameraState , const Vector3& hitPos ,
		const Vector3& rayDir);

	Color3 getDirectIllumination(SubPathState& cameraState ,
		const Vector3& hitPos , BSDF& bsdf);

	Color3 connectVertices(PathVertex& lightVertex , 
		BSDF& cameraBsdf , const Vector3& cameraHitPos ,
		SubPathState& cameraState);
	/************************/

	Real mis(Real pdf);
};

#endif
