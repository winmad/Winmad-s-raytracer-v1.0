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
		const VertexCM& vertexCM;
		const Vector3& cameraPos;
		const BSDF& cameraBsdf;
		const SubPathState& cameraState;
		Color3 contrib;

		RangeQuery(const VertexCM& vertexCM , 
			const Vector3& cameraPos , const BSDF& cameraBsdf ,
			const SubPathState& cameraState)
			: vertexCM(vertexCM) , cameraPos(cameraPos) ,
			cameraBsdf(cameraBsdf) , cameraState(cameraState) ,
			contrib(0) {}

		void process(const PathVertex& lightVertex)
		{
			if (lightVertex.pathLength + cameraState.pathLength > vertexCM.maxPathLength ||
				lightVertex.pathLength + cameraState.pathLength < vertexCM.minPathLength)
				return;

			Vector3 lightDir = lightVertex.bsdf.wiWorld();

			Real cosCamera , cameraBsdfDirPdf , cameraBsdfRevPdf;

			Color3 cameraBsdfFactor = cameraBsdf.f(vertexCM.scene , 
				lightDir , cosCamera , &cameraBsdfDirPdf , &cameraBsdfRevPdf);

		}
	};

	int minPathLength , maxPathLength;
	Real baseRadius;         // initial merging radius
	Real radiusAlpha;        // radius reduction per iteration
	Real misVmWeightFactor;
	Real misVcWeightFactor;
	Real samplesPerPixel;
	Real lightSubPathNum;
	Real vmNormalization;    // 1 / (PI * radius ^ 2 * lightSubPathNum)

	std::vector<PathVertex> lightVertices;
	std::vector<int> pathEnds;

	VertexKDtree tree;
};

#endif