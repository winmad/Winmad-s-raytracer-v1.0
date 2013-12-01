#ifndef MULTIPLE_MERGE_H
#define MULTIPLE_MERGE_H

#include "surfaceIntegrator.h"

struct MMPathState
{
	Vector3 origin , pos;
	Vector3 dir;
	Color3 throughput;
	BSDF bsdf;

	int pathLength : 30;
	int specularPath : 1;
	int isFiniteLight : 1;

	Real curPdf , culmPdf;
};

class LightSubPath
{
public:
	Vector3 origin; // origin
	Vector3 pos; // origin -> nextPos
	Color3 throughput;
	Color3 dirContrib , indirContrib;

	Vector3 wo; // dir at origin
	BSDF bsdf; // bsdf at nextPos

	Real lastPdf , curPdf;

    LightSubPath() {}
    
	LightSubPath(MMPathState& oldPathState , MMPathState& curPathState)
	{	
		origin = oldPathState.pos;
		pos = curPathState.pos;
		
		throughput.r = curPathState.throughput.r / oldPathState.throughput.r;
		throughput.g = curPathState.throughput.g / oldPathState.throughput.g;
		throughput.b = curPathState.throughput.b / oldPathState.throughput.b;

		dirContrib = indirContrib = Color3(0.f);
		
		bsdf = curPathState.bsdf;
		
		curPdf = curPathState.culmPdf;
	}
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

		void process(LightSubPath& lightSubPath)
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
            
			Real weightFactor = 0.f;

			Real glossyIndex = cameraSubPath.bsdf.glossyIndex;

            weightFactor = multipleMerge.mergeFactor(glossyIndex)
					/ (1.f + multipleMerge.mergeFactor(glossyIndex));

            weightFactor /= multipleMerge.mergeKernel * cameraSubPath.culmPdf;
            //weightFactor /= multipleMerge.mergeKernel / PI * 0.5f;
            
			Color3 totContrib = lightSubPath.dirContrib + lightSubPath.indirContrib;
			contrib = contrib + (cameraBsdfFactor | totContrib) * weightFactor;
		}
	};

	class MergeQuery
	{
	public:
		MultipleMerge& multipleMerge;
		LightSubPath& lightSubPath;
		Color3 contrib;
		int mergeNum;

		MergeQuery(MultipleMerge& multipleMerge ,
			LightSubPath& lightSubPath)
			: multipleMerge(multipleMerge) , lightSubPath(lightSubPath) , 
			contrib(0) , mergeNum(0) {}

		void process(LightSubPath& subPath)
		{
			Color3 totContrib = subPath.dirContrib + subPath.indirContrib;
			if (totContrib.isBlack())
				return;

			Real cosTerm , bsdfDirPdf , bsdfRevPdf;
			Color3 bsdfFactor = subPath.bsdf.f(multipleMerge.scene ,
				lightSubPath.wo , cosTerm , &bsdfDirPdf , &bsdfRevPdf);

			if (bsdfFactor.isBlack())
				return;

			Vector3 dist = subPath.pos - lightSubPath.origin;

			Real weightFactor = 0.f;

			Real glossyIndex = subPath.bsdf.glossyIndex;

			if (cmp(dist.length()) == 0)
			{
				// connect
				weightFactor = 1.f / (1.f + multipleMerge.mergeFactor(glossyIndex));
			}
			else
			{
				// merge
				weightFactor = multipleMerge.mergeFactor(glossyIndex)
					/ (1.f + multipleMerge.mergeFactor(glossyIndex));
				weightFactor /= multipleMerge.mergeKernel * lightSubPath.lastPdf;
                //weightFactor /= multipleMerge.mergeKernel / PI * 0.5f;
			}
			
			contrib = contrib + (bsdfFactor | totContrib) * weightFactor;
		}
	};

	int minPathLength , maxPathLength;
	int lightPathNum , cameraPathNum , pixelNum;
	int iterations;
	Real baseRadius;         // initial merging radius
	Real radiusAlpha;        // radius reduction per iteration
	Real radius;
	Real mergeKernel;        // constant kernel

	std::vector<LightSubPath> lightSubPaths;

	std::vector<int> lightSubPathIndex;

	KdTree<LightSubPath> *lightTree;

	MultipleMerge() {}

	void init(char *filename , Parameters& para);

	void runIteration(int iter);

	void render();

	void outputImage(char *filename);

	void generateLightSample(MMPathState& lightState);

	Color3 connectToCamera(MMPathState& lightState , const Vector3& hitPos ,
		BSDF& bsdf);

	bool sampleScattering(BSDF& bsdf , const Vector3& hitPos , 
		MMPathState& pathState);

	Vector3 generateCameraSample(const int pathIndex , 
		MMPathState& cameraState);

	Color3 getLightRadiance(AbstractLight *light , MMPathState& cameraState ,
		const Vector3& hitPos , const Vector3& rayDir);

	Color3 getDirectIllumination(MMPathState& cameraState , const Vector3& hitPos , BSDF& bsdf);

	Color3 connectVertices(LightSubPath& lightSubPath , BSDF& bsdf , 
		const Vector3& hitPos , MMPathState& cameraState);

	Real mis(Real pdf)
	{
		return pdf;
	}

	Real connectFactor(Real glossyIndex)
	{
		return exp(-glossyIndex);
	}

	Real mergeFactor(Real glossyIndex)
	{
		return PI * SQR(radius) * exp(-glossyIndex);
	}
};

#endif
