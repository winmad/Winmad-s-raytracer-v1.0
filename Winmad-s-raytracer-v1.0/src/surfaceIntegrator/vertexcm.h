#ifndef VERTEX_CM_H
#define VERTEX_CM_H

#include "surfaceIntegrator.h"
#include "vertexKDtree.h"

struct SubPathState
{
	Vector3 pathOrigin;
	Vector3 dir;
	Color3 throughput;
	int pathLength : 30;
	int isFiniteLight : 1;
	int specularPath : 1;

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

		RangeQuery(VertexCM& vertexCM , 
			Vector3& cameraPos , BSDF& cameraBsdf ,
			SubPathState& cameraState)
			: vertexCM(vertexCM) , cameraPos(cameraPos) ,
			cameraBsdf(cameraBsdf) , cameraState(cameraState) ,
			contrib(0) {}

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

			cameraBsdfDirPdf *= cameraBsdf.continueProb;
			cameraBsdfRevPdf *= lightVertex.bsdf.continueProb;

			Real weightLight = lightVertex.dVCM * vertexCM.misVcWeightFactor +
				lightVertex.dVM * vertexCM.mis(cameraBsdfDirPdf);

			Real weightCamera = cameraState.dVCM * vertexCM.misVcWeightFactor +
				cameraState.dVM * vertexCM.mis(cameraBsdfRevPdf);

			Real misWeight = 1.f / (weightLight + 1.f + weightCamera);

			contrib = contrib + (cameraBsdfFactor | lightVertex.throughput) *
				misWeight;
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

	VertexKDtree tree;

	VertexCM() {}

	void init(char *filename , Parameters& para);

	void runIteration(int iter);

	void generateLightSample(SubPathState& lightState);

	Real mis(Real pdf);
};

#endif