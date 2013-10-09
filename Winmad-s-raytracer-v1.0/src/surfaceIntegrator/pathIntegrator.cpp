#include "pathIntegrator.h"

void PathIntegrator::init(char *filename , Parameters& para)
{
	maxTracingDepth = para.MAX_TRACING_DEPTH;
	samplesPerPixel = para.SAMPLES_PER_PIXEL;
	samplesOfLight = para.SAMPLES_OF_LIGHT;
	samplesOfHemisphere = para.SAMPLES_OF_HEMISPHERE;

	scene.init(filename , para);

	height = para.HEIGHT; width = para.WIDTH;
}

// MIS power = 1, balance heuristic
static Real mis(Real pdf)
{
	return pdf;
}

// MIS weight for 2 pdfs
static Real mis(Real samplePdf , Real otherPdf)
{
	return mis(samplePdf) / (mis(samplePdf) + mis(otherPdf));
}

Color3 PathIntegrator::raytracing(const Ray& ray , int dep)
{
	Intersection inter;
	Color3 pathWeight(1.f);
	Color3 res(0.f);

	int pathLength = 1;
	bool lastSpecular = 1;
	Real lastPdf = 1.f;
		
	Real lightPickProb = 1.f / scene.lights.size();

	Ray r(ray);

	for (;; pathLength++)
	{
		if (scene.intersect(r , inter) == NULL)
			break;

		Vector3 hitPoint = r(inter.t);
		BSDF bsdf(-r.dir , inter , scene);
		if (!bsdf.isValid())
			break;

		// hit light directly
		if (bsdf.matId < 0)
		{
			AbstractLight *light = scene.lights[-bsdf.matId - 1];
			Real directPdfArea;
			Color3 contrib = light->getRadiance(scene.sceneSphere ,
				r.dir , hitPoint , &directPdfArea);
			if (contrib.isBlack())
				break;

			Real misWeight = 1.f;
			if (pathLength > 1 && !lastSpecular)
			{
				Real directPdf = pdfAtoW(directPdfArea , 
					inter.t , bsdf.cosWi());
				misWeight = mis(lastPdf , directPdf / scene.lights.size());
			}

			res = res + (pathWeight | contrib) * misWeight;
			break;
		}

		// direct illumination estimation
		if (!bsdf.isDelta)
		{
			int lightId = (int)(rng.randFloat() * scene.lights.size());
			AbstractLight *light = scene.lights[lightId];

			Vector3 dirToLight;
			Real dist , directPdf;

			Color3 illu = light->illuminance(scene.sceneSphere , 
				hitPoint , rng.randVector3() , dirToLight , dist ,
				directPdf);

			if (!illu.isBlack() && 
				!scene.occluded(hitPoint , dirToLight , 
				hitPoint + dirToLight * dist))
			{
				Real bsdfPdf , cosWo;
				Color3 bsdfFactor = bsdf.f(scene , dirToLight , cosWo ,
					&bsdfPdf);
				if (!bsdfFactor.isBlack())
				{
					Real weight = 1.f;
					if (!light->isDelta())
					{
						Real contProb = bsdf.continueProb;
						bsdfPdf *= contProb;
						weight = mis(directPdf / scene.lights.size() ,
							bsdfPdf);
					}

					Color3 contrib = (illu | bsdfFactor) * 
						(weight * cosWo / (lightPickProb * bsdfPdf));

					res = res + (contrib | pathWeight);
				}
			}
		}

		// continue random walk
		Real pdf , cosWo;
		int sampledType;

		Color3 bsdfFactor = bsdf.sample(scene , rng.randVector3() ,
			r.dir , pdf , cosWo , &sampledType);

		if (bsdfFactor.isBlack())
			break;

		// Russian Roulette
		Real contProb = bsdf.continueProb;
		lastSpecular = ((sampledType & BSDF_SPECULAR) != 0);
		lastPdf = pdf * contProb;

		if (cmp(contProb - 1.f) < 0)
		{
			if (cmp(rng.randFloat() - contProb) > 0)
				break;
			pdf *= contProb;
		}

		pathWeight = (pathWeight | bsdfFactor) * (cosWo / pdf);

		r.origin = hitPoint + r.dir * EPS;
	}
	return res;
}
