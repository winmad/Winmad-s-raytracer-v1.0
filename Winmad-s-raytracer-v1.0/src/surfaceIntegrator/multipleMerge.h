#ifndef MULTIPLE_MERGE_H
#define MULTIPLE_MERGE_H

#include "surfaceIntegrator.h"

struct MMPathState
{
	Vector3 origin , pos;
	Vector3 dir;
	Color3 throughput;
	BSDF bsdf;

	Color3 dirContrib , indirContrib;
	Vector3 posAtOrigin , dirAtOrigin;

	int pathLength : 30;
	int specularPath : 1;
	int isFiniteLight : 1;
};

class MultipleMerge : public SurfaceIntegrator
{
public:
	class GatherQuery
	{
	public:
		MultipleMerge& multipleMerge;
		MMPathState& cameraSubPath;
		Color3 contrib;
		int mergeNum;

		GatherQuery(MultipleMerge& multipleMerge ,
			MMPathState& cameraSubPath)
			: multipleMerge(multipleMerge) , cameraSubPath(cameraSubPath) ,
			contrib(0) , mergeNum(0) {}

		void process(MMPathState& lightSubPath)
		{
			Vector3 lightDir = lightSubPath.bsdf.wiWorld();

			Real cosCamera , cameraBsdfDirPdf , cameraBsdfRevPdf;

			Color3 cameraBsdfFactor = cameraSubPath.bsdf.f(multipleMerge.scene , 
				lightDir , cosCamera , &cameraBsdfDirPdf , &cameraBsdfRevPdf);

			if (cameraBsdfFactor.isBlack())
				return;

			mergeNum++;

			cameraBsdfDirPdf *= cameraSubPath.bsdf.continueProb;
			cameraBsdfRevPdf *= lightSubPath.bsdf.continueProb;
            
			Real weightFactor = multipleMerge.mergeFactor() /
				(multipleMerge.connectFactor(cameraBsdfRevPdf , cameraSubPath.bsdf.glossyIndex) + 
				multipleMerge.mergeFactor());
			weightFactor *= multipleMerge.mergeKernel;

			Color3 totContrib = lightSubPath.dirContrib + lightSubPath.indirContrib;
			contrib = contrib + (cameraBsdfFactor | totContrib) * weightFactor;
		}
	};

	class MergeQuery
	{
	public:
		MultipleMerge& multipleMerge;
		MMPathState& lightSubPath;
		Color3 contrib;
		int mergeNum;

		MergeQuery(MultipleMerge& multipleMerge ,
			MMPathState& lightSubPath)
			: multipleMerge(multipleMerge) , lightSubPath(lightSubPath) , 
			contrib(0) , mergeNum(0) {}

		void process(MMPathState& subPath)
		{
			Color3 totContrib = subPath.dirContrib + subPath.indirContrib;
			if (totContrib.isBlack())
				return;

			Real cosTerm , bsdfDirPdf , bsdfRevPdf;
			Color3 bsdfFactor = subPath.bsdf.f(multipleMerge.scene ,
				lightSubPath.dirAtOrigin , cosTerm , &bsdfDirPdf , &bsdfRevPdf);

			if (bsdfFactor.isBlack())
				return;

			Vector3 dist = subPath.pos - lightSubPath.origin;

			bsdfDirPdf *= subPath.bsdf.continueProb;

			Real weightFactor = 0.f;

			Real glossyIndex = subPath.bsdf.glossyIndex;

			Real pdf = bsdfDirPdf;

// 			if (cmp(dist.length()) == 0)
// 			{
// 				// connect
// 				weightFactor = multipleMerge.connectFactor(pdf) / 
// 					(multipleMerge.connectFactor(pdf) + 
// 					multipleMerge.mergeFactor());
// 
// 				weightFactor *= cosTerm / bsdfDirPdf;
// 			}
// 			else
			{
				// merge
				weightFactor = multipleMerge.mergeFactor()
					/ (multipleMerge.connectFactor(pdf , subPath.bsdf.glossyIndex) + 
					multipleMerge.mergeFactor());
				weightFactor *= multipleMerge.mergeKernel;
			}
			
			mergeNum++;

			contrib = contrib + (bsdfFactor | totContrib) * weightFactor;
		}
	};

	int minPathLength , maxPathLength;
	int lightPathNum , cameraPathNum , interPathNum;
	int partialPathNum , pixelNum;
	int iterations;
	Real baseRadius;         // initial merging radius
	Real radiusAlpha;        // radius reduction per iteration
	Real radius;
	Real mergeKernel;        // constant kernel

	std::vector<MMPathState> lightSubPaths;

	KdTree<MMPathState> *lightTree;

	std::vector<std::vector<int> > prevSubPaths;

	MultipleMerge() {}

	void init(char *filename , Parameters& para);

	void preparation(double mergeRadius);

	void runIteration(int iter);

	void render();

	void outputImage(char *filename);

	void generateLightSample(MMPathState& lightState);

	void generateInterSample(MMPathState& interState);

	Color3 connectToCamera(MMPathState& lightState , const Vector3& hitPos ,
		BSDF& bsdf);

	bool sampleScattering(BSDF& bsdf , const Vector3& hitPos , 
		MMPathState& pathState , Real *_bsdfDirPdf , 
		bool isCameraPath);

	Vector3 generateCameraSample(const int pathIndex , 
		MMPathState& cameraState);

	Color3 getLightRadiance(AbstractLight *light , MMPathState& cameraState ,
		const Vector3& hitPos , const Vector3& rayDir);

	Color3 getDirectIllumination(MMPathState& cameraState , const Vector3& hitPos , BSDF& bsdf);

	Color3 connectVertices(MMPathState& lightSubPath , BSDF& bsdf , 
		const Vector3& hitPos , MMPathState& cameraState);

	Real mis(Real pdf)
	{
		return pdf;
	}

	Real connectFactor(Real pdf , Real glossyIndex)
	{
		return mis(pdf);
	}

	Real mergeFactor()
	{
		return mis(0.5 * SQR(radius) * partialPathNum / scene.totArea);
	}

	Real gatherFactor(Real mergeNum , Real glossyIndex)
	{
		Real v1 = 0.1f , v2 = 0.9f;
		Real res = mergeNum / (mergeNum + std::exp(glossyIndex));
		return (res - v1) / (v2 - v1);
	}
};

#endif
