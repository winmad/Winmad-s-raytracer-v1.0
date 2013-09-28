#include "whitted.h"

void WhittedIntegrator::init(char *filename , Parameters& para)
{
	maxTracingDepth = para.MAX_TRACING_DEPTH;	
	
	samplesPerPixel = para.SAMPLES_PER_PIXEL;

	scene.init(filename , para);
	viewPort = scene.viewPort;

	height = para.HEIGHT; width = para.WIDTH;

	viewPort.delta.x = (viewPort.r.x - viewPort.l.x) / (Real)width;
	viewPort.delta.y = (viewPort.r.y - viewPort.l.y) / (Real)height;
	viewPort.delta.z = 0.0;
}

//static FILE *fp = fopen("debug_whitted.txt" , "w");

Color3 WhittedIntegrator::raytracing(const Ray& ray , int dep)
{
	if (dep > maxTracingDepth)
		return Color3(0.0 , 0.0 , 0.0);

	Intersection inter;
	Geometry *g = NULL;

	g = this->scene.intersect(ray , inter);

	if (g == NULL)
		return Color3(0.0 , 0.0 , 0.0);

	Color3 phongRes , reflectRes , transRes;
	Vector3 reflectDir , transDir;
	Ray reflectRay , transRay;

	Ray shadowRay = Ray(scene.lightlist[0].pos ,
		inter.p - scene.lightlist[0].pos);
	Real directCoe = scene.shadowRayTest(shadowRay , inter.p);

	Vector3 wi , wo;
	wi = scene.lightlist[0].pos - inter.p;
	wi.normalize();
	wo = -ray.dir;
	wo.normalize();

	Color3 brdf = g->getMaterial().bxdf->calcBrdf(wi , wo , inter.n);
	phongRes = scene.lightlist[0].color | brdf;
	phongRes = phongRes * clampVal(wi ^ inter.n , 0.0 , 1.0);

	if (g->getMaterial().shininess > 0)
	{
		reflectDir = getReflectDir(wo , inter.n);
		reflectRay = Ray(inter.p + reflectDir * (2 * EPS) , reflectDir);
		reflectRes = raytracing(reflectRay , dep + 1);
	}

	if (g->getMaterial().transparency > 0 &&
		cmp(g->getMaterial().refractionIndex) != 0)
	{
		transDir = getTransDir(wo , inter.n , 
			g->getMaterial().refractionIndex , inter.inside);
		if (transDir.isNormal())
		{
			transRay = Ray(inter.p + transDir * (2 * EPS) , transDir);
			transRes = raytracing(transRay , dep + 1);
		}
		else
		{
			transRes = Color3(0.0 , 0.0 , 0.0);
		}
	}

	Color3 res;
	res = phongRes * (1 - inter.inside) + 
		reflectRes * g->getMaterial().shininess +
		transRes * g->getMaterial().transparency;
    /*
    fprintf(fp , "reflect = ");
    print_color3(fp , reflectRes);
    fprintf(fp , " trans = ");
    print_color3(fp , transRes);
    fprintf(fp , "\n");
    */
	return res;
}
