#include "pathIntegrator.h"

void PathIntegrator::init(char *filename , Parameters& para)
{
	maxTracingDepth = para.MAX_TRACING_DEPTH;
	samplesPerPixel = para.SAMPLES_PER_PIXEL;
	samplesOfLight = para.SAMPLES_OF_LIGHT;
	samplesOfHemisphere = para.SAMPLES_OF_HEMISPHERE;

	scene.init(filename , para);
	viewPort = scene.viewPort;

	height = para.HEIGHT; width = para.WIDTH;

	viewPort.delta.x = (viewPort.r.x - viewPort.l.x) / (Real)width;
	viewPort.delta.y = (viewPort.r.y - viewPort.l.y) / (Real)height;
	viewPort.delta.z = 0.0;
}

static Color3 directIllumination(PathIntegrator& pathIntegrator , 
						   Geometry* g , const Vector3& p , 
						   const Vector3& n , const Vector3& wo)
{
    Color3 res = Color3(0.0 , 0.0 , 0.0);
    Vector3 wi;
    Real cosine;
    Color3 brdf;
    Ray ray;
	Intersection inter;
    
    for (int i = 0; i < pathIntegrator.samplesOfLight; i++)
    {
        int k = rand() % pathIntegrator.scene.lightlist.size();
        wi = pathIntegrator.scene.lightlist[k].pos - p;
        wi.normalize();

        ray = Ray(pathIntegrator.scene.lightlist[k].pos - wi * (eps * 10.0) ,
			-wi);
        if (cmp(pathIntegrator.scene.shadowRayTest(ray , p)) == 0)
			continue;
        
        brdf = g->getMaterial().bxdf->calcBrdf(wi , wo , n);

        //cosTerm = clamp_val(n ^ lightDir , 0.0 , 1.0);
        cosine = 1.0;

        res = res + (pathIntegrator.scene.lightlist[k].color | brdf) *
            cosine;
    }
    res = res / pathIntegrator.samplesOfLight;
    return res;
}

//static FILE *fp = fopen("debug.txt" , "w");

static Color3 indirectIllumination(PathIntegrator& pathIntegrator, Geometry* g ,
                             const Vector3& p , const Vector3& n ,
                             const Vector3& wo , int dep)
{
    Color3 res = Color3(0.0 , 0.0 , 0.0);
    Vector3 wi;
    Ray ray;
    Color3 tmp , brdf;

    Real absorb = 1.0 - (g->getMaterial().bxdf->ks.r + 
		g->getMaterial().bxdf->ks.g + g->getMaterial().bxdf->ks.b) / 3.0;

    if (cmp(drand48() - absorb) <= 0)
        return res;

    Color3 reflectRes = Color3(0.0 , 0.0 , 0.0);
    
    for (int i = 0; i < pathIntegrator.samplesOfHemisphere; i++)
    {
        wi = sampleDirOnHemisphere(n);
        ray = Ray(p + wi * (eps * 10.0) , wi);
        tmp = pathIntegrator.raytracing(ray , dep + 1);

        brdf = g->getMaterial().bxdf->calcBrdf(wi , wo , n);

        reflectRes = reflectRes + (tmp | brdf);
    }
    reflectRes = reflectRes / pathIntegrator.samplesOfHemisphere * PI;
    
    res = reflectRes;
    
    return res / (1.0 - absorb);
}

Color3 PathIntegrator::raytracing(const Ray& ray , int dep)
{
	if (dep > maxTracingDepth)
		return Color3(0.0 , 0.0 , 0.0);

	Geometry *g = NULL;
	Intersection inter;

	g = scene.intersect(ray , inter);

	if (g == NULL)
		return Color3(0.0 , 0.0 , 0.0);

    // transmission
    Color3 transRes = Color3(0.0 , 0.0 , 0.0);
    
    if (cmp(g->getMaterial().transparency) > 0 &&
        cmp(g->getMaterial().refractionIndex) != 0)
    {
        Vector3 transDir = getTransDir(-ray.dir , inter.n , 
			g->getMaterial().refractionIndex , inter.inside);
        
        if (transDir.isNormal())
        {
            Ray transRay = Ray(inter.p + transDir * (eps * 10.0) , transDir);
            transRes = raytracing(transRay , dep + 1);
            transRes = transRes * g->getMaterial().transparency;
        }
    }
    
	Color3 emit , direct , indirect , res;
    emit = Color3(0 , 0 , 0);
    direct = directIllumination(*this , g , inter.p , inter.n , -ray.dir);
    indirect = indirectIllumination(*this , g , inter.p , inter.n , -ray.dir , dep);
    res = emit + direct + indirect + transRes;
    return res;
}
