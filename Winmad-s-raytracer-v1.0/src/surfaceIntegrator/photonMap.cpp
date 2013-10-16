#include "photonMap.h"
#include "../sampler/sampler.h"
#include <opencv2/opencv.hpp>

//static FILE *fp = fopen("debug_pm.txt" , "w");

void PhotonIntegrator::init(char *filename , Parameters& para)
{
    nCausticPhotons = 10000;
    
    nIndirectPhotons = 100000;

    knnPhotons = 50;

    maxSqrDis = 0.1;
    
    maxPathLength = 5;

    maxPhotonShot = 500000;

    causticMap = NULL;
    indirectMap = NULL;

    maxTracingDepth = para.MAX_TRACING_DEPTH;
    samplesPerPixel = para.SAMPLES_PER_PIXEL;

    scene.init(filename , para);

	height = para.HEIGHT; width = para.WIDTH;

	film = new ImageFilm(height , width);
}

void PhotonIntegrator::outputImage(char *filename)
{
	film->outputImage(filename , 1.f / 1000.f , 2.2);
}

void PhotonIntegrator::buildPhotonMap(Scene& scene)
{
    if (scene.lights.size() <= 0)
        return;

    causticPhotons.clear();
    indirectPhotons.clear();
    
    bool causticDone = 0 , indirectDone = 0;
    int nShot = 0;

    bool specularPath;
    int nIntersections;
    Intersection inter;
    Real lightPickProb = 1.f / scene.lights.size();

    while (!causticDone || !indirectDone)
    {
        nShot++;

        /* generate initial photon ray */
        int k = (int)(rng.randFloat() * scene.lights.size());

		Vector3 lightPos , lightDir;
		Real emissionPdf , directPdfArea , cosAtLight;
		Color3 alpha;

		AbstractLight *l = scene.lights[k];
		alpha = l->emit(scene.sceneSphere , rng.randVector3() , rng.randVector3() ,
			lightPos , lightDir , emissionPdf , &directPdfArea ,
			&cosAtLight);

		alpha = alpha * cosAtLight / (emissionPdf * lightPickProb);
        
		Ray photonRay(lightPos , lightDir);

        if (!alpha.isBlack())
        {
            specularPath = 1;
            nIntersections = 0;
            Geometry *g = scene.intersect(photonRay , inter);
            while (g != NULL && inter.matId > 0)
            {
				BSDF bsdf(-photonRay.dir , inter , scene);

                nIntersections++;
                bool hasNonSpecular = (cmp(bsdf.componentProb.diffuseProb) > 0 ||
                                      cmp(bsdf.componentProb.glossyProb) > 0);
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
				Real pdf , cosWo;
				int sampledType;

				Color3 bsdfFactor = bsdf.sample(scene , rng.randVector3() ,
					photonRay.dir , pdf , cosWo , &sampledType);

				if (bsdfFactor.isBlack())
					break;

				if (sampledType & BSDF_NON_SPECULAR)
					specularPath = 0;

				// Russian Roulette
				Real contProb = bsdf.continueProb;
				
				if (cmp(contProb - 1.f) < 0)
				{
					if (cmp(rng.randFloat() - contProb) > 0)
						break;
					pdf *= contProb;
				}

				alpha = (alpha | bsdfFactor) * (cosWo / pdf);

				photonRay.origin = inter.p + photonRay.dir * EPS;
				photonRay.tmin = 0.f; photonRay.tmax = INF;

                g = scene.intersect(photonRay , inter);
            }
        }
    }
}

static Color3 estimate(PhotonKDtree *map , int nPaths , int knn ,
                                  Scene& scene , Intersection& inter , 
								  const Vector3& wo , Real maxSqrDis)
{
    Color3 res = Color3(0.0 , 0.0 , 0.0);
    
    if (map == NULL)
        return res;
    
    std::vector<ClosePhoton> kPhotons;
    Photon photon;
    photon.p = inter.p;

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
    if (cmp(wo ^ inter.n) < 0)
        nv = -inter.n;
    else
        nv = inter.n;

    Real scale = 1.0 / (PI * msd * nFoundPhotons);
    
	BSDF bsdf(wo , inter , scene);
	Real cosTerm , pdf;
    for (int i = 0; i < nFoundPhotons; i++)
    {
        /*
        Real k = kernel(kPhotons[i].photon , p , msd);
        k = 1.0 / PI;
        Real scale = k / (nPaths * msd);
        */
        if (cmp(nv ^ kPhotons[i].photon->wi) > 0)
        {
            Color3 brdf = bsdf.f(scene , kPhotons[i].photon->wi , 
				cosTerm , &pdf);
			if (brdf.isBlack())
				continue;
            res = res + (brdf | kPhotons[i].photon->alpha) * 
				(cosTerm * scale / pdf);
        }
        else
        {
			Vector3 wi(kPhotons[i].photon->wi);
			wi.z *= -1.f;
			Color3 brdf = bsdf.f(scene , wi , cosTerm , &pdf);
			if (brdf.isBlack())
				continue;
			res = res + (brdf | kPhotons[i].photon->alpha) * 
				(cosTerm * scale / pdf);
        }
    }
    return res;
}

static Color3 directIllumination(Scene& scene , Intersection& inter ,
                           RNG& rng , const Vector3& visionDir)
{
    Color3 res = Color3(0.0 , 0.0 , 0.0);
    Vector3 lightDir;
    Real cosine;
    Color3 brdf;
    Ray ray;
    
	BSDF bsdf(visionDir , inter , scene);

	int N = 1;
    for (int i = 0; i < N; i++)
    {
        int k = rand() % scene.lights.size();
		AbstractLight *l = scene.lights[k];
		
		Vector3 dirToLight;
		Real dist , directPdf;

		Color3 illu = l->illuminance(scene.sceneSphere , inter.p , rng.randVector3() ,
			dirToLight , dist , directPdf);

		illu = illu / (directPdf / scene.lights.size());

        if (scene.occluded(inter.p + dirToLight * EPS , dirToLight ,
			inter.p + dirToLight * (inter.t - EPS)))
            continue;
        
		Real bsdfPdf;
        brdf = bsdf.f(scene , dirToLight , cosine , &bsdfPdf);
		if (brdf.isBlack())
			continue;
        res = res + (illu | brdf) * (cosine / bsdfPdf);
    }
    res = res / N;
    return res;
}

static Color3 finalGathering(PhotonKDtree *map , Scene& scene , 
							 Intersection& inter , RNG& rng , const Vector3& wo ,
                             int gatherSamples , int knn , Real maxSqrDis)
{
    Color3 res = Color3(0.0 , 0.0 , 0.0);
    for (int i = 0; i < gatherSamples; i++)
    {
		Real pdf;
        Vector3 wi = sampleCosHemisphere(rng.randVector3() , &pdf);
        Ray ray = Ray(inter.p + wi * EPS , wi);

        Intersection _inter;
        
        Geometry *_g = scene.intersect(ray , _inter);

        if (_g == NULL)
            continue;
        
        Color3 tmp = estimate(map , 0 , knn , scene , _inter , -wi , maxSqrDis);

		BSDF bsdf(wi , _inter , scene);
		Real cosine , bsdfPdf;
        Color3 brdf = bsdf.f(scene , wo , cosine , &bsdfPdf);
		if (brdf.isBlack())
			continue;
		pdf *= bsdfPdf;
        res = res + (tmp | brdf) * (cosine / pdf);
    }
    res = res / gatherSamples;
    return res;
}

Color3 PhotonIntegrator::raytracing(const Ray& ray , int dep)
{
    Color3 res = Color3(0.0 , 0.0 , 0.0);

    if (dep > 2)
        return res;

    Geometry *g = NULL;
	Intersection inter;
    Ray reflectRay , transRay;

    g = scene.intersect(ray , inter);

    if (g == NULL)
        return res;

	if (inter.matId < 0)
	{
		AbstractLight *l = scene.lights[-inter.matId - 1];
		return l->getIntensity() * 100.f;
	}
    
    res = res + directIllumination(scene , inter , rng , -ray.dir);
      
    res = res + estimate(indirectMap , nIndirectPaths , knnPhotons , scene , inter , -ray.dir , maxSqrDis);

	// final gathering is too slow!

    //res = res + finalGathering(indirectMap , scene , inter , rng , -ray.dir , 50 , knnPhotons , maxSqrDis);
    
    res = res + estimate(causticMap , nCausticPaths , knnPhotons , scene , inter , -ray.dir , maxSqrDis);

	BSDF bsdf(-ray.dir , inter , scene);

	Real pdf , cosWo;
	int sampledType;
	Ray newRay;

	Color3 bsdfFactor = bsdf.sample(scene , rng.randVector3() ,
		newRay.dir , pdf , cosWo , &sampledType);

	if (bsdfFactor.isBlack())
		return res;

	// Russian Roulette
	Real contProb = bsdf.continueProb;

	if (cmp(contProb - 1.f) < 0)
	{
		if (cmp(rng.randFloat() - contProb) > 0)
			return res;
		pdf *= contProb;
	}
    
	newRay.origin = inter.p + newRay.dir * EPS;
	newRay.tmin = 0; newRay.tmax = INF;

	Color3 contrib = raytracing(newRay , dep + 1);

	res = res + (contrib | bsdfFactor) * (cosWo / pdf);

    return res;
}

void PhotonIntegrator::visualize(const std::vector<Photon>& photons ,
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
        z0 = 0;
        Vector3 dir = p - scene.camera.pos;
        if (cmp(dir.z) != 0)
        {
            t = (z0 - p.z) / dir.z;
            x = p.x + t * dir.x;
            y = p.y + t * dir.y;
        }
        else
        {
            t = INF;
            x = 0;
            y = 0;
        }

        Ray shadowRay = Ray(p - dir * (10.0 * EPS) , -dir);
        if (scene.intersect(shadowRay) != NULL)
            continue;
        
		Vector3 posWorld(x , y , z0);
		Vector3 posRaster = scene.camera.worldToRaster.tPoint(posWorld);
        int r , c;
        r = posRaster.y;
		c = posRaster.x;
        r = std::max(0 , std::min(r , height - 1));
        c = std::max(0 , std::min(c , width - 1));
        cnt[r][c]++;
        col[r][c] = col[r][c] + photon.alpha;
    }

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
			//scale
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
