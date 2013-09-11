#include "IGIIntegrator.h"

void IGIIntegrator::init(char *filename , Parameters& para)
{
	threshold = 0.1;

	maxTracingDepth = para.MAX_TRACING_DEPTH;
	samplesPerPixel = para.SAMPLES_PER_PIXEL;
	samplesOfHemisphere = para.SAMPLES_OF_HEMISPHERE;

	numDirectLight = NUM_DIRECT_LIGHT;
	numIndirectLight = NUM_INDIRECT_LIGHT;

	scene.init(filename , para);
	viewPort = scene.viewPort;

	height = para.HEIGHT; width = para.WIDTH;

	viewPort.delta.x = (viewPort.r.x - viewPort.l.x) / (Real)width;
	viewPort.delta.y = (viewPort.r.y - viewPort.l.y) / (Real)height;
	viewPort.delta.z = 0.0;
}

void IGIIntegrator::generateVirtualLights()
{
	// generate direct lights
	for (int i = 0; i < scene.lightlist.size(); i++)
	{
		PointLight l = scene.lightlist[i];
		VirtualLight vl = VirtualLight(l.pos , l.color);
		_directLights.push_back(vl);
	}
	// generate indirect virtual lights
	int n = 0;
	Intersection inter;
	while (n < numIndirectLight)
	{
		int k = rand() % (int)_directLights.size();
		Color3 alpha = _directLights[k].alpha;

		Vector3 dir = uniformSampleDirOnSphere();
		Ray ray = Ray(_directLights[k].p , dir);

		if (cmp(alpha.intensity() - threshold) >= 0)
		{
			int nIntersections = 0;
			Geometry *g = scene.intersect(ray , inter);
			while (g != NULL)
			{
				nIntersections++;
				bool hasNonSpecular = (cmp(g->getMaterial().shininess) == 0 &&
					cmp(g->getMaterial().transparency) == 0);
				// store indirect virtual lights
				if (hasNonSpecular)
				{
					Color3 contrib = (g->getMaterial().bxdf->calcRho(-ray.dir , inter.n) |
						alpha) / PI;
					Vector3 reflectDir = getReflectDir(-ray.dir , inter.n);
					VirtualLight vl = VirtualLight(inter.p + reflectDir * 10.0 * eps ,
						contrib);
					_indirectLights.push_back(vl);
					n++;
					if (n >= numIndirectLight)
						break;
				}

				if (nIntersections > 5)
					break;

				/* find new ray direction */
                /* handle specular reflection and transmission first */
                Vector3 dir;
                if (cmp(g->getMaterial().shininess) > 0)
                {
                    dir = getReflectDir(-ray.dir , inter.n);
                    Color3 brdf = g->getMaterial().bxdf->calcBrdf(-ray.dir , 
						dir , inter.n);
                    Real cosine = fabs(inter.n ^ ray.dir);
                    alpha = (alpha | brdf) * cosine;
                }
                else if (cmp(g->getMaterial().transparency) > 0)
                {
                    dir = getTransDir(-ray.dir , inter.n , 
						g->getMaterial().refractionIndex , inter.inside);
                    if (!dir.isNormal())
                        break;
                    Color3 btdf = g->getMaterial().bxdf->calcBtdf(-ray.dir , 
						dir , -inter.n);
                    Real cosine = fabs(inter.n ^ ray.dir);
                    alpha = (alpha | btdf) * cosine;
                }
                else
                {
                    /* handle non-specular reflection by cosine sampling on
                       hemisphere */
                    dir = sampleDirOnHemisphere(inter.n);
                    Color3 brdf = g->getMaterial().bxdf->calcBrdf(-ray.dir , 
						dir , inter.n);
                    alpha = (alpha | brdf) * PI;
                }

				if (cmp(alpha.intensity() - threshold) < 0)
					break;
				ray = Ray(inter.p + dir * (10.0 * eps) , dir);

                /* Possibly terminate by Russian Roulette */
                if (nIntersections > 3)
                {
                    if (cmp(drand48() - 0.5) <= 0)
                        break;
                    alpha = alpha / 0.5;
                }
                g = scene.intersect(ray , inter);
			}
		}
	}

	directLights.init(_directLights);
	directLights.buildTree(directLights.root);
	indirectLights.init(_indirectLights);
	indirectLights.buildTree(indirectLights.root);
}

static Color3 approxIllumination(const std::vector<VirtualLight>& lights ,
	Scene& scene , Geometry *g , const Vector3& p , 
	const Vector3& n , const Vector3& wo)
{
	Color3 res = Color3(0.0 , 0.0 , 0.0);
	Vector3 wi;
	Real cosine;
	Color3 brdf;
	Ray ray;
	Intersection inter;

	for (int i = 0; i < lights.size(); i++)
	{
		wi = lights[i].p - p;
		wi.normalize();

		ray = Ray(lights[i].p - wi * (eps * 10.0) , -wi);
		if (cmp(scene.shadowRayTest(ray , p)) == 0)
			continue;

		brdf = g->getMaterial().bxdf->calcBrdf(wi , wo , n);

		cosine = clampVal(n ^ wi , 0.0 , 1.0);

		res = res + (lights[i].alpha | brdf) *
			cosine;
	}
	res = res / (Real)lights.size();
	return res;
}

static Color3 approxIllumination(LightTree& lights ,
	Scene& scene , Geometry *g , const Vector3& p , 
	const Vector3& n , const Vector3& wo)
{
	std::priority_queue<LightCluster> cut;
	Color3 res = lights.findLightCuts(p , n , wo , g , scene , cut);
	
	res = res / (Real)cut.size();
	return res;
	
}

Color3 IGIIntegrator::raytracing(const Ray& ray , int dep)
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
	
	res = res + approxIllumination(directLights , 
			scene , g , inter.p , inter.n , -ray.dir);
	
	res = res + approxIllumination(indirectLights , 
			scene , g , inter.p , inter.n , -ray.dir);
	
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