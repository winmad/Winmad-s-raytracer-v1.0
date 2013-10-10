#ifndef PHOTON_MAP_H
#define PHOTON_MAP_H

#include "../scene/scene.h"
#include "../parameters.h"
#include "photonKDtree.h"
#include "surfaceIntegrator.h"

static std::vector<Photon> causticPhotons;
static std::vector<Photon> indirectPhotons;

class PhotonIntegrator : public SurfaceIntegrator
{
public:
    int nCausticPhotons , nIndirectPhotons;
    int knnPhotons;
    Real maxSqrDis;
    int maxPathLength , maxPhotonShot;

    int nCausticPaths , nIndirectPaths;
    PhotonKDtree *causticMap;
    PhotonKDtree *indirectMap;

	int maxTracingDepth;

    PhotonIntegrator() {}
    
    void init(char *filename , Parameters& para);
    
    void buildPhotonMap(Scene& scene);

    Color3 raytracing(const Ray& ray , int dep);
    
    void visualize(const std::vector<Photon>& photons , 
    			   Scene& scene , char *filename);
};

#endif
