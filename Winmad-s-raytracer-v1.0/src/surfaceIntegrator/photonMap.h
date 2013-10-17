#ifndef PHOTON_MAP_H
#define PHOTON_MAP_H

#include "../scene/scene.h"
#include "../parameters.h"
#include "surfaceIntegrator.h"
#include "../math/color.h"
#include "../math/vector.h"
#include <vector>

class Photon
{
public:
	Vector3 pos , wi;
	Color3 alpha;

	Photon() {}

	Photon(const Vector3& _p , const Vector3& _wi , const Color3& _alpha)
		: pos(_p) , wi(_wi) , alpha(_alpha) {}
};

class ClosePhoton
{
public:
	Photon *photon;
	Real sqrDis;

	ClosePhoton(Photon *p = NULL , Real _sqrDis = INF)
	{
		photon = p;
		sqrDis = _sqrDis;
	}

	bool operator <(const ClosePhoton &p) const
	{
		return (sqrDis < p.sqrDis);
	}
};

class ClosePhotonQuery
{
public:
	int K;
	Real maxSqrDis;
	std::vector<ClosePhoton> kPhotons;

	ClosePhotonQuery(int knn , Real _maxSqrDis)
	{
		K = knn;
		maxSqrDis = _maxSqrDis;
		kPhotons.clear();
	}

	void init(Real _maxSqrDis)
	{
		maxSqrDis = _maxSqrDis;
		kPhotons.clear();
	}

	void process(Photon *photon , Real sqrDis)
	{
		if (kPhotons.size() < K)
		{
			kPhotons.push_back(ClosePhoton(photon , sqrDis));
			if (kPhotons.size() == K)
			{
				std::make_heap(kPhotons.begin() , kPhotons.end());
				maxSqrDis = kPhotons[0].sqrDis;
			}
		}
		else
		{
			std::pop_heap(kPhotons.begin() , kPhotons.end());
			kPhotons[K - 1] = ClosePhoton(photon , sqrDis);
			std::push_heap(kPhotons.begin() , kPhotons.end());
			maxSqrDis = kPhotons[0].sqrDis;
		}
	}
};

class PhotonIntegrator : public SurfaceIntegrator
{
public:
    int nCausticPhotons , nIndirectPhotons;
    int knnPhotons;
    Real maxSqrDis;
    int maxPathLength , maxPhotonShot;

    int nCausticPaths , nIndirectPaths;

	KdTree<Photon> *causticMap;
	KdTree<Photon> *indirectMap;

	std::vector<Photon> causticPhotons;
	std::vector<Photon> indirectPhotons;

	int maxTracingDepth;

    PhotonIntegrator() {}
    
    void init(char *filename , Parameters& para);
    
    void buildPhotonMap(Scene& scene);

    Color3 raytracing(const Ray& ray , int dep);
    
    void visualize(const std::vector<Photon>& photons , 
    			   Scene& scene , char *filename);

	void outputImage(char *filename);
};

#endif
