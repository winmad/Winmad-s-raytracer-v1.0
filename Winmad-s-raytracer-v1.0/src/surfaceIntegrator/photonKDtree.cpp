#include "photonKDtree.h"

static Real get_projection(const Vector3& v , int axis)
{
	if (axis == 0)
		return v.x;
	if (axis == 1)
		return v.y;
	if (axis == 2)
		return v.z;
	return inf;
}

static int cmp_sort_point(const void *a , const void *b)
{
    PhotonPointSorted p1 = *(PhotonPointSorted*)a;
    PhotonPointSorted p2 = *(PhotonPointSorted*)b;
    return cmp(p1.pos - p2.pos);
}

void PhotonKDtree::init(const std::vector<Photon>& photons)
{
    totNum = photons.size();
    depMax = (int)(1.2 * log((double)totNum) + 2.0);

    root = new PhotonKDtreeNode();
    root->photons.clear();
    for (int i = 0; i < photons.size(); i++)
        root->photons.push_back(photons[i]);

    for (int i = 0; i < 3; i++)
    {
        root->p[i] = new PhotonPointSorted[totNum];

        for (int j = 0; j < totNum; j++)
        {
            root->p[i][j].pos = get_projection(photons[j].p , i);
            root->p[i][j].index = j;
        }

        qsort(root->p[i] , totNum , sizeof(PhotonPointSorted) , cmp_sort_point);
    }

    root->axis = -1;
    root->left = root->right = NULL;
}

void PhotonKDtree::buildTree(PhotonKDtreeNode *tr , int dep)
{
    if (dep > depMax)
        return;
    if (tr->photons.size() <= 1)
        return;

    /* find median */
    int axis = dep % 3;
    int mid = tr->photons.size() / 2;

    tr->axis = axis;
    
    tr->left = new PhotonKDtreeNode();
    tr->right = new PhotonKDtreeNode();

    tr->div = new int[tr->photons.size()];

    for (int i = 0; i < mid; i++)
        tr->div[tr->p[axis][i].index] = 0;
    for (int i = mid + 1; i < tr->photons.size(); i++)
        tr->div[tr->p[axis][i].index] = 1;
    tr->div[tr->p[axis][mid].index] = -1;

    PhotonKDtreeNode *left = tr->left , *right = tr->right;
    int N = tr->photons.size();

    left->axis = right->axis = -1;
    left->left = left->right = right->left = right->right = NULL;

    int *index_to_l , *index_to_r;

    index_to_l = new int[N];
    index_to_r = new int[N];
    
    for (int i = 0; i < mid; i++)
    {
        left->photons.push_back(tr->photons[tr->p[axis][i].index]);
        index_to_l[tr->p[axis][i].index] = i;
    }
    for (int i = mid + 1; i < N; i++)
    {
        right->photons.push_back(tr->photons[tr->p[axis][i].index]);
        index_to_r[tr->p[axis][i].index] = i - mid - 1;
    }

    for (axis = 0; axis < 3; axis++)
    {
    	int lp = 0 , rp = 0;
        left->p[axis] = new PhotonPointSorted[mid + 1];
        right->p[axis] = new PhotonPointSorted[N - mid + 1];
        for (int i = 0; i < N; i++)
        {
            if (tr->div[tr->p[axis][i].index] == 0)
            {
                left->p[axis][lp].pos = tr->p[axis][i].pos;
                left->p[axis][lp].index = index_to_l[tr->p[axis][i].index];
                lp++;
            }
            else if (tr->div[tr->p[axis][i].index] == 1)
            {
                right->p[axis][rp].pos = tr->p[axis][i].pos;
                right->p[axis][rp].index = index_to_r[tr->p[axis][i].index];
                rp++;
            }
        }
    }

    Photon photon = tr->photons[tr->p[tr->axis][mid].index];
    tr->photons.clear();
    tr->photons.push_back(photon);
    for (int i = 0; i < 3; i++)
        delete[] tr->p[i];
    delete[] tr->div;
    delete[] index_to_l;
    delete[] index_to_r;
    
    buildTree(left , dep + 1);
    buildTree(right , dep + 1);
}

void PhotonKDtree::naiveSearchKPhotons(std::vector<ClosePhoton>& kPhotons ,
                                   PhotonKDtreeNode *tr , const Photon& p ,
                                   const int& K , Real& maxSqrDis)
{
    if (tr == NULL)
        return;
    if (tr->photons.size() <= 0)
        return;

    Vector3 d = p.p - tr->photons[0].p;
    Real sqrDis = d.sqrLength();
    
    if (cmp(sqrDis - maxSqrDis) < 0)
    {
        if (kPhotons.size() < K)
        {
            kPhotons.push_back(ClosePhoton(&tr->photons[0] , sqrDis));
            if (kPhotons.size() == K)
            {
                std::make_heap(kPhotons.begin() , kPhotons.end());
                maxSqrDis = kPhotons[0].sqrDis;
            }
        }
        else
        {
            std::pop_heap(kPhotons.begin() , kPhotons.end());
            kPhotons[K - 1] = ClosePhoton(&tr->photons[0] , sqrDis);
            std::push_heap(kPhotons.begin() , kPhotons.end());
            maxSqrDis = kPhotons[0].sqrDis;
        }
    }
    naiveSearchKPhotons(kPhotons , tr->left , p , K , maxSqrDis);
    naiveSearchKPhotons(kPhotons , tr->right , p , K , maxSqrDis);
}

void PhotonKDtree::searchKPhotons(std::vector<ClosePhoton>& kPhotons ,
                                    PhotonKDtreeNode *tr , const Photon& p ,
                                    const int& K , Real& maxSqrDis)
{
    if (tr == NULL)
        return;
    if (tr->photons.size() <= 0)
        return;
    
    PhotonKDtreeNode *near , *far;
    Real p_pos = get_projection(p.p , tr->axis);
    Real split_pos = get_projection(tr->photons[0].p , tr->axis);
    Real delta = p_pos - split_pos;
    Real sqrDelta = SQR(delta);
    
    if (cmp(delta) <= 0)
    {
        near = tr->left;
        far = tr->right;
    }
    else
    {
        near = tr->right;
        far = tr->left;
    }
    
    searchKPhotons(kPhotons , near , p , K , maxSqrDis);

    Vector3 d = p.p - tr->photons[0].p;
    Real sqrDis = d.sqrLength();
    
    if (cmp(sqrDis - maxSqrDis) < 0)
    {
        if (kPhotons.size() < K)
        {
            kPhotons.push_back(ClosePhoton(&tr->photons[0] , sqrDis));
            if (kPhotons.size() == K)
            {
                std::make_heap(kPhotons.begin() , kPhotons.end());
                maxSqrDis = kPhotons[0].sqrDis;
            }
        }
        else
        {
            std::pop_heap(kPhotons.begin() , kPhotons.end());
            kPhotons[K - 1] = ClosePhoton(&tr->photons[0] , sqrDis);
            std::push_heap(kPhotons.begin() , kPhotons.end());
            maxSqrDis = kPhotons[0].sqrDis;
        }
    }
    if (cmp(sqrDelta - maxSqrDis) < 0)
        searchKPhotons(kPhotons , far , p , K , maxSqrDis);
}
