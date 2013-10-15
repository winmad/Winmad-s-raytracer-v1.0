#include "whitted.h"

void WhittedIntegrator::init(char *filename , Parameters& para)
{
	maxTracingDepth = para.MAX_TRACING_DEPTH;	
	
	samplesPerPixel = para.SAMPLES_PER_PIXEL;

	scene.init(filename , para);

	height = para.HEIGHT; width = para.WIDTH;

	film = new ImageFilm(height , width);
}

//static FILE *fp = fopen("debug_whitted.txt" , "w");

static Color3 directIllumination(WhittedIntegrator& integrator ,
	const Intersection& inter , const Vector3& wo)
{
	Color3 res = Color3(0);
	Real dist , directPdf , emissionPdf , cosAtLight;
	Vector3 wi;

	int LIGHT_SAMPLE_NUM = 1;
	for (int i = 0; i < LIGHT_SAMPLE_NUM; i++)
	{
		int k = rand() % integrator.scene.lights.size();

		Color3 illu = integrator.scene.lights[k]->illuminance(
			integrator.scene.sceneSphere , inter.p , 
			integrator.rng.randVector3() , wi , dist ,
			directPdf , &emissionPdf , &cosAtLight);

		if (integrator.scene.occluded(inter.p , wi , inter.p + wi * dist))
			continue;

		BSDF bsdf(wi , inter , integrator.scene);
		Real cosWo;

		Color3 tmp = (illu * cosAtLight / emissionPdf)
			| bsdf.f(integrator.scene , wo , cosWo);

		res = res + tmp * cosWo;
	}
	res = res / LIGHT_SAMPLE_NUM;
	return res;
}

Color3 WhittedIntegrator::raytracing(const Ray& ray , int dep)
{
	if (dep > maxTracingDepth)
		return Color3(0.0 , 0.0 , 0.0);

	Intersection inter;
	Geometry *g = NULL;

	g = this->scene.intersect(ray , inter);

	if (g == NULL)
		return Color3(0.0 , 0.0 , 0.0);

	if (inter.matId < 0)
	{
		AbstractLight *l = scene.lights[-inter.matId - 1];
		return l->getRadiance(scene.sceneSphere , ray.dir , inter.p);
	}

	Color3 directRes , reflectRes , transRes;
	Vector3 reflectDir , transDir;
	Ray reflectRay , transRay;
	
	BSDF bsdf(-ray.dir , inter , scene);

	directRes = directIllumination(*this , inter , -ray.dir) *
		(bsdf.componentProb.diffuseProb + bsdf.componentProb.glossyProb);
	
	if (cmp(bsdf.componentProb.reflectProb) > 0)
	{
		reflectDir = getReflectDir(-ray.dir , inter.n);
		reflectRay = Ray(inter.p + reflectDir * EPS , reflectDir);
		reflectRes = raytracing(reflectRay , dep + 1);
	}

	if (cmp(bsdf.componentProb.transProb) > 0 &&
		cmp(scene.materials[bsdf.matId].index) > 0)
	{
		transDir = getTransDir(-ray.dir , inter.n , 
			scene.materials[bsdf.matId].index , inter.inside);
		if (transDir.isNormal())
		{
			transRay = Ray(inter.p + transDir * EPS , transDir);
			transRes = raytracing(transRay , dep + 1);
		}
		else
		{
			transRes = Color3(0.0 , 0.0 , 0.0);
		}
	}

	Color3 res;
	res = directRes + 
		reflectRes * bsdf.componentProb.reflectProb +
		transRes * bsdf.componentProb.transProb;
    /*
    fprintf(fp , "reflect = ");
    print_color3(fp , reflectRes);
    fprintf(fp , " trans = ");
    print_color3(fp , transRes);
    fprintf(fp , "\n");
    */
	return res;
}
