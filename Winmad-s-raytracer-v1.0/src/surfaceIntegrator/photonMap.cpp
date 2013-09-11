#include "photonMap.h"
#include "../sampler/sampler.h"
#include <opencv2/opencv.hpp>

//static FILE *fp = fopen("debug_pm.txt" , "w");

void PhotonIntegrator::init(char *filename , Parameters& para)
{
    nCausticPhotons = 20000;
    
    nIndirectPhotons = 100000;

    knnPhotons = 50;

    maxSqrDis = 1000;
    
    maxPathLength = 5;

    maxPhotonShot = 500000;

    causticMap = NULL;
    indirectMap = NULL;

    maxTracingDepth = para.MAX_TRACING_DEPTH;
    samplesPerPixel = para.SAMPLES_PER_PIXEL;

    scene.init(filename , para);
    viewPort = scene.viewPort;

	height = para.HEIGHT; width = para.WIDTH;

	viewPort.delta.x = (viewPort.r.x - viewPort.l.x) / (Real)width;
	viewPort.delta.y = (viewPort.r.y - viewPort.l.y) / (Real)height;
	viewPort.delta.z = 0.0;
}

void PhotonIntegrator::buildPhotonMap(Scene& scene)
{
    if (scene.lightlist.size() <= 0)
        return;

    causticPhotons.clear();
    indirectPhotons.clear();
    
    bool causticDone = 0 , indirectDone = 0;
    int nShot = 0;

    bool specularPath;
    int nIntersections;
    Intersection inter;
    
    while (!causticDone || !indirectDone)
    {
        nShot++;

        /* generate initial photon ray */
        int k = rand() % (int)scene.lightlist.size();
        Color3 alpha = scene.lightlist[k].color;
        
        Vector3 dir = uniformSampleDirOnSphere();
        Ray photonRay = Ray(scene.lightlist[k].pos , dir);

        alpha = alpha * (4.0 * PI) * (scene.lightlist.size());

        if (!alpha.isBlack())
        {
            specularPath = 1;
            nIntersections = 0;
            Geometry *g = scene.intersect(photonRay , inter);
            while (g != NULL)
            {
                nIntersections++;
                bool hasNonSpecular = (cmp(g->getMaterial().shininess) == 0 &&
                                      cmp(g->getMaterial().transparency) == 0);
                if (hasNonSpecular)
                {
                    Photon photon(inter.p , -photonRay.dir , alpha);
                    if (specularPath && nIntersections > 1)
                    {
                        if (!causticDone)
                        {
                            causticPhotons.push_back(photon);
                            if (causticPhotons.size() == nCausticPhotons)
                            {
                                visualize(causticPhotons , viewPort ,
                                          scene , "caustic_photon_map.bmp");
                                
                                causticDone = 1;
                                nCausticPaths = nShot;
                                causticMap = new PhotonKDtree();
                                causticMap->init(causticPhotons);
                                causticMap->buildTree(causticMap->root , 0);
                            }
                        }
                    }
                    else
                    {
                        if (nIntersections > 1 && !indirectDone)
                        {
                            indirectPhotons.push_back(photon);
                            if (indirectPhotons.size() == nIndirectPhotons)
                            {
                                visualize(indirectPhotons , viewPort ,
                                          scene , "indirect_photon_map.bmp");
                                
                                indirectDone = 1;
                                nIndirectPaths = nShot;
                                
                                indirectMap = new PhotonKDtree();
                                indirectMap->init(indirectPhotons);
                                indirectMap->buildTree(indirectMap->root , 0);
                            }
                        }
                    }
                }
                if (nIntersections > maxPathLength)
                    break;

                /* find new photon ray direction */
                /* handle specular reflection and transmission first */
                Vector3 dir;
                if (cmp(g->getMaterial().shininess) > 0)
                {
                    dir = getReflectDir(-photonRay.dir , inter.n);
                    Color3 brdf = g->getMaterial().bxdf->calcBrdf(-photonRay.dir , 
						dir , inter.n);
                    Real cosine = fabs(inter.n ^ photonRay.dir);
                    alpha = (alpha | brdf) * cosine;
                }
                else if (cmp(g->getMaterial().transparency) > 0)
                {
                    dir = getTransDir(-photonRay.dir , inter.n , 
						g->getMaterial().refractionIndex , inter.inside);
                    if (!dir.isNormal())
                        break;
                    Color3 btdf = g->getMaterial().bxdf->calcBtdf(-photonRay.dir , 
						dir , -inter.n);
                    Real cosine = fabs(inter.n ^ photonRay.dir);
                    alpha = (alpha | btdf) * cosine;
                }
                else
                {
                    /* handle non-specular reflection by cosine sampling on
                       hemisphere */
                    dir = sampleDirOnHemisphere(inter.n);
                    Color3 brdf = g->getMaterial().bxdf->calcBrdf(-photonRay.dir , 
						dir , inter.n);
                    alpha = (alpha | brdf) * PI;
                    specularPath = 0;
                }

				if (alpha.isBlack())
					break;
				photonRay = Ray(inter.p + dir * (10.0 * eps) , dir);

                /* Possibly terminate by Russian Roulette */
                if (nIntersections > 3)
                {
                    if (cmp(drand48() - 0.5) <= 0)
                        break;
                    alpha = alpha / 0.5;
                }
                g = scene.intersect(photonRay , inter);
            }
        }
    }
}

static Color3 estimate(PhotonKDtree *map , int nPaths , int knn ,
                                  Geometry* g , const Vector3& p ,
                                  const Vector3& n , const Vector3& wo ,
                                  Real maxSqrDis)
{
    Color3 res = Color3(0.0 , 0.0 , 0.0);
    
    if (map == NULL)
        return res;
    
    std::vector<ClosePhoton> kPhotons;
    Photon photon;
    photon.p = p;

    Real searchSqrDis = maxSqrDis;
    Real msd; /* max square distance */
    while (kPhotons.size() < knn)
    {
        msd = searchSqrDis;
        kPhotons.clear();
        map->searchKPhotons(kPhotons , map->root , photon , knn , msd);
        searchSqrDis *= 2.0;
    }
    
    int nFoundPhotons = kPhotons.size();

    if (nFoundPhotons == 0)
        return res;
    
    Vector3 nv;
    if (cmp(wo ^ n) < 0)
        nv = -n;
    else
        nv = n;

    Real scale = 1.0 / (PI * msd * nFoundPhotons);
    
    for (int i = 0; i < nFoundPhotons; i++)
    {
        /*
        Real k = kernel(kPhotons[i].photon , p , msd);
        k = 1.0 / PI;
        Real scale = k / (nPaths * msd);
        */
        if (cmp(nv ^ kPhotons[i].photon->wi) > 0)
        {
            Color3 brdf = g->getMaterial().bxdf->calcBrdf(
				kPhotons[i].photon->wi , wo , nv);
            res = res + (brdf | kPhotons[i].photon->alpha) * scale;
        }
        else
        {
            Color3 btdf = g->getMaterial().bxdf->calcBtdf(
				kPhotons[i].photon->wi , wo , nv);
            res = res + (btdf | kPhotons[i].photon->alpha) * scale;
        }
    }
    return res;
}

static Color3 directIllumination(Scene& scene , Geometry* g ,
                           const Vector3& p , const Vector3& n ,
                           const Vector3& visionDir)
{
    Color3 res = Color3(0.0 , 0.0 , 0.0);
    Vector3 lightDir;
    Real cosine;
    Color3 brdf;
    Ray ray;
    Intersection inter;
    
    for (int i = 0; i < 8; i++)
    {
        int k = rand() % scene.lightlist.size();
        lightDir = scene.lightlist[k].pos - p;
        lightDir.normalize();

        ray = Ray(scene.lightlist[k].pos - lightDir * (eps * 10.0) , -lightDir);
        if (cmp(scene.shadowRayTest(ray , p)) == 0)
            continue;
        
        brdf = g->getMaterial().bxdf->calcBrdf(lightDir , visionDir , n);

        cosine = clampVal(n ^ lightDir , 0.0 , 1.0);

        res = res + (scene.lightlist[k].color | brdf) *
            cosine;
    }
    res = res / 8;
    return res;
}

static Color3 finalGathering(PhotonKDtree *map , Scene& scene , Geometry *g ,
                             const Vector3& p , const Vector3& n , const Vector3& wo ,
                             int gatherSamples , int knn , Real maxSqrDis)
{
    Color3 res = Color3(0.0 , 0.0 , 0.0);
    for (int i = 0; i < gatherSamples; i++)
    {
        Vector3 wi = sampleDirOnHemisphere(n);
        Ray ray = Ray(p + wi * (10.0 * eps) , wi);

        Intersection inter;
        
        Geometry *_g = scene.intersect(ray , inter);

        if (_g == NULL)
            continue;
        
        Color3 tmp = estimate(map , 0 , knn , _g , inter.p , inter.n , -wi , maxSqrDis);

        Color3 brdf = g->getMaterial().bxdf->calcBrdf(wi , wo , n);
        res = res + (tmp | brdf) * PI;
    }
    res = res / gatherSamples;
    return res;
}

Color3 PhotonIntegrator::raytracing(const Ray& ray , int dep)
{
    Color3 res = Color3(0.0 , 0.0 , 0.0);

    if (dep > maxTracingDepth)
        return res;

    Geometry *g = NULL;
	Intersection inter;
    Ray reflectRay , transRay;

    g = scene.intersect(ray , inter);

    if (g == NULL)
        return res;
    
    res = res + directIllumination(scene , g , inter.p , inter.n , -ray.dir);
      
    //res = res + estimate(indirectMap , nIndirectPaths , knnPhotons , g , p , n , -ray.dir , maxSqrDis);

    //Color3 t1 = estimate(indirectMap , nIndirectPaths , knnPhotons , g , p , n , -ray.dir , maxSqrDis);
    //Color3 t2 = finalGathering(indirectMap , scene , g , p , n , -ray.dir , 50 , knnPhotons , maxSqrDis);
    
    res = res + finalGathering(indirectMap , scene , g , inter.p , inter.n , -ray.dir , 50 , knnPhotons , maxSqrDis);
    
    res = res + estimate(causticMap , nCausticPaths , knnPhotons , g , inter.p , inter.n , -ray.dir , maxSqrDis);

    if (cmp(g->getMaterial().shininess) > 0)
    {
        Vector3 dir = getReflectDir(-ray.dir , inter.n);
        reflectRay = Ray(inter.p + dir * (10.0 * eps) , dir);
        res = res + raytracing(reflectRay , dep + 1);
    }

    if (cmp(g->getMaterial().transparency) > 0)
    {
        Vector3 dir = getTransDir(-ray.dir , inter.n , g->getMaterial().refractionIndex , inter.inside);
        if (dir.isNormal())
        {
            transRay = Ray(inter.p + dir * (10.0 * eps) , dir);
            res = res + raytracing(transRay , dep + 1);
        }
    }
    
    return res;
}

void PhotonIntegrator::visualize(const std::vector<Photon>& photons ,
                                 const ViewPort& viewPort ,
                                 Scene& scene , char *filename)
{
    IplImage *img = 0;
    img = cvCreateImage(cvSize(width , height) ,
                        IPL_DEPTH_8U , 3);
    Color3 res;

    Color3 **col;
    int **cnt;

    col = new Color3*[height];
    cnt = new int*[height];
    for (int i = 0; i < height; i++)
    {
        col[i] = new Color3[width];
        cnt[i] = new int[width];
        for (int j = 0; j < width; j++)
        {
            col[i][j] = Color3(0.0 , 0.0 , 0.0);
            cnt[i][j] = 0;
        }
    }

    for (int i = 0; i < photons.size(); i++)
    {
        Photon photon = photons[i];
        Vector3 p = photon.p;
        Real x , y , z0 , t;
        z0 = viewPort.l.z;
        Vector3 dir = p - scene.camera;
        if (cmp(dir.z) != 0)
        {
            t = (z0 - p.z) / dir.z;
            x = p.x + t * dir.x;
            y = p.y + t * dir.y;
        }
        else
        {
            t = inf;
            x = 0;
            y = 0;
        }

        Ray shadowRay = Ray(p - dir * (10.0 * eps) , -dir);
        if (scene.intersect(shadowRay) != NULL)
            continue;
        
        int r , c;
        r = (int)(((y - viewPort.l.y) / (viewPort.r.y - viewPort.l.y) * (double)height));
        c = (int)(((x - viewPort.l.x) / (viewPort.r.x - viewPort.l.x) * (double)width));
        r = std::max(0 , std::min(r , height - 1));
        c = std::max(0 , std::min(c , width - 1));
        cnt[r][c]++;
        col[r][c] = col[r][c] + photon.alpha;
    }

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            col[i][j] = col[i][j] / ((double)cnt[i][j] * 4 * PI * 100);
            int h = img->height;
			int w = img->width;
			int step = img->widthStep;
			int channels = img->nChannels;

			uchar *data = (uchar*)img->imageData;
			data[(h - i - 1) * step + j * channels + 0] = col[i][j].B();
			data[(h - i - 1) * step + j * channels + 1] = col[i][j].G();
			data[(h - i - 1) * step + j * channels + 2] = col[i][j].R();
        }
    }

    for (int i = 0; i < height; i++)
    {
        delete[] col[i];
        delete[] cnt[i];
    }
    delete[] col;
    delete[] cnt;
    cvSaveImage(filename , img);
}
