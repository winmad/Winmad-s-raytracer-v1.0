#ifndef PATH_REUSING_H
#define PATH_REUSING_H

#include "surfaceIntegrator.h"

struct PathState
{
	Vector3 origin , pos;
	Vector3 dir;
	Color3 throughput;
	BSDF bsdf;

	int pathLength : 30;
	int specularPath : 1;
	int isFiniteLight : 1;

	Real dVCM , dVC , dVM;
};

class SubPath
{
public:
	Vector3 pos; // origin
	Vector3 nextPos; // origin -> nextPos
	Color3 throughput;
	Color3 lightContrib , pathContrib;

	Vector3 wo; // dir at origin
	BSDF bsdf; // bsdf at nextPos
	int rasterX , rasterY;
	bool startFlag;

	SubPath() : rasterX(-1) , rasterY(-1) {}

	SubPath(PathState& oldPathState , PathState& curPathState)
	{
		pos = oldPathState.pos;

		throughput.r = curPathState.throughput.r / oldPathState.throughput.r;
		throughput.g = curPathState.throughput.g / oldPathState.throughput.g;
		throughput.b = curPathState.throughput.b / oldPathState.throughput.b;

		lightContrib = pathContrib = Color3(0.f);
		rasterX = rasterY = -1;

		startFlag = 0;
	}

	bool isStart()
	{
		return startFlag;
	}
};

class PathReusing : public SurfaceIntegrator
{
public:
	class RangeQuery
	{
	public:
		PathReusing& pathReusing;
		Vector3& cameraPos;
		BSDF& cameraBsdf;
		PathState& cameraSubPath;
		Color3 contrib;
		int mergeNum;

		RangeQuery(PathReusing& pathReusing , 
			Vector3& cameraPos , BSDF& cameraBsdf ,
			PathState& cameraSubPath)
			: pathReusing(pathReusing) , cameraPos(cameraPos) ,
			cameraBsdf(cameraBsdf) , cameraSubPath(cameraSubPath) ,
			contrib(0) , mergeNum(0) {}

		void process(PathState& lightVertex)
		{
			Vector3 lightDir = lightVertex.bsdf.wiWorld();

			Real cosCamera , cameraBsdfDirPdf , cameraBsdfRevPdf;

			Color3 cameraBsdfFactor = cameraBsdf.f(pathReusing.scene , 
				lightDir , cosCamera , &cameraBsdfDirPdf , &cameraBsdfRevPdf);

			if (cameraBsdfFactor.isBlack())
				return;

			mergeNum++;

			cameraBsdfDirPdf *= cameraBsdf.continueProb;
			cameraBsdfRevPdf *= lightVertex.bsdf.continueProb;

			Real wLight = lightVertex.dVCM * pathReusing.misVcWeightFactor +
				lightVertex.dVM * pathReusing.mis(cameraBsdfDirPdf);
			Real wCamera = cameraSubPath.dVCM * pathReusing.misVcWeightFactor +
				cameraSubPath.dVM * pathReusing.mis(cameraBsdfRevPdf);
			Real weight = 1.f / (wLight + 1.f + wCamera);

			contrib = contrib + (cameraBsdfFactor | lightVertex.throughput) * weight;
		}
	};

	class MergeQuery
	{
	public:
		PathReusing& pathReusing;
		Vector3& pathEnd;
		SubPath& cameraSubPath;
		Color3 contrib;
		int mergeNum;

		MergeQuery(PathReusing& pathReusing , Vector3& pathEnd ,
			SubPath& cameraSubPath)
			: pathReusing(pathReusing) , pathEnd(pathEnd) ,
			cameraSubPath(cameraSubPath) , contrib(0) ,
			mergeNum(0) {}

		void process(SubPath& subPath)
		{
			Color3 totContrib = subPath.lightContrib + subPath.pathContrib;
			if (totContrib.isBlack())
				return;

			Real cosTerm , bsdfDirPdf , bsdfRevPdf;
			Color3 bsdfFactor = cameraSubPath.bsdf.f(pathReusing.scene ,
				subPath.wo , cosTerm , &bsdfDirPdf , &bsdfRevPdf);

			if (bsdfFactor.isBlack())
				return;

			bsdfDirPdf *= cameraSubPath.bsdf.continueProb;
			bsdfRevPdf *= cameraSubPath.bsdf.continueProb;

			contrib = contrib + totContrib;
		}
	};

	int minPathLength , maxPathLength;
	int lightPathNum , cameraPathNum , pixelNum;
	int iterations;
	Real baseRadius;         // initial merging radius
	Real radiusAlpha;        // radius reduction per iteration
	Real vmNormalization;
	Real misVcWeightFactor;
	Real misVmWeightFactor;

	std::vector<PathState> lightStates;
	std::vector<SubPath> cameraSubPaths;

	std::vector<int> lightStateIndex;

	KdTree<PathState> *lightTree;
	KdTree<SubPath> *pathTree;
	
	PathReusing() {}

	void init(char *filename , Parameters& para);

	void runIteration(int iter);

	void render();

	void outputImage(char *filename);

	void generateLightSample(PathState& lightState);

	Color3 connectToCamera(PathState& lightState , const Vector3& hitPos ,
		BSDF& bsdf);

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
