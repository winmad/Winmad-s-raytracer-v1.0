#ifndef PHOTON_KDTREE_H
#define PHOTON_KDTREE_H

#include "../math/color.h"
#include "../math/vector.h"
#include <vector>

class Photon
{
public:
    Vector3 p , wi;
    Color3 alpha;

    Photon() {}

    Photon(const Vector3& _p , const Vector3& _wi , const Color3& _alpha)
        : p(_p) , wi(_wi) , alpha(_alpha) {}
};

class ClosePhoton
{
public:
    Photon *photon;
    Real sqrDis;

    ClosePhoton() {}

    ClosePhoton(Photon *p = NULL , Real _sqrDis = inf)
    {
        photon = p;
        sqrDis = _sqrDis;
    }

    bool operator <(const ClosePhoton &p) const
    {
        return (cmp(sqrDis - p.sqrDis) < 0);
    }
};

struct PhotonPointSorted
{
    Real pos;
    int index;
};

class PhotonKDtreeNode
{
public:
    int axis;
    
    std::vector<Photon> photons;

    PhotonPointSorted *p[3];
    int *div;

    PhotonKDtreeNode *left , *right;

    PhotonKDtreeNode()
        : left(NULL) , right(NULL) {}
};

class PhotonKDtree
{
public:
    int totNum;
    
    int depMax;

    PhotonKDtreeNode *root;

    PhotonKDtree()
        : root(NULL) {}

    void init(const std::vector<Photon>& photons);

    void buildTree(PhotonKDtreeNode *tr , int dep);

	void naiveSearchKPhotons(std::vector<ClosePhoton>& kPhotons ,
                                PhotonKDtreeNode *tr , const Photon& p ,
                                const int& K , Real& maxSqrDis);
                                   
    void searchKPhotons(std::vector<ClosePhoton>& kPhotons ,
                          PhotonKDtreeNode *tr , const Photon& p ,
                          const int& K , Real& maxSqrDis);
};

#endif
