#ifndef BSDF_H
#define BSDF_H

#include "../math/color.h"
#include "../math/vector.h"
#include "../math/frame.h"
#include "../geometry/intersection.h"
#include "../scene/scene.h"
#include "../sampler/sampler.h"
#include "material.h"

// glossy: use phong model
enum BSDFType
{
	BSDF_REFLECTION = 1 << 0,
	BSDF_TRANSMISSION = 1 << 1,

	BSDF_DIFFUSE = 1 << 2,
	BSDF_GLOSSY = 1 << 3,
	
	BSDF_SPECULAR = BSDF_REFLECTION |
					BSDF_TRANSMISSION,

	BSDF_NON_SPECULAR = BSDF_DIFFUSE |
						BSDF_GLOSSY,

	BSDF_ALL = BSDF_SPECULAR | 
			   BSDF_NON_SPECULAR
};

class BSDF
{
public:
	BSDFType type;

	int matId;

	Frame localFrame;

	Vector3 wiLocal;

	bool isDelta; // true if material is purely specular

	Real continueProb; // used for Russian roulette

	Real fresnelReflect; // Fresnel reflection coefficient

	struct ComponentProb
	{
		Real diffuseProb;
		Real glossyProb;
		Real reflectProb;
		Real transProb;
	};

	ComponentProb componentProb;

	BSDF() : matId(0) {}

	BSDF(const Vector3& wi , const Intersection& inter ,
		const Scene& scene) 
	{
		init(wi , inter , scene);
	}

	void init(const Vector3& wi , const Intersection& inter ,
		const Scene& scene)
	{
		matId = 0;
		localFrame.buildFromZ(inter.n);
		wiLocal = localFrame.worldToLocal(wi);

		if (cmp(wiLocal.z) == 0)
			return;

		if (inter.matId > 0)
		{
			const Material& mat = scene.materials[inter.matId];
			calcComponentProb(mat , componentProb);
		}

		isDelta = (cmp(componentProb.diffuseProb) == 0 &&
			cmp(componentProb.glossyProb) == 0);

		matId = inter.matId;
	}

	bool isValid()
	{
		return matId != 0;
	}

	Real cosWi()
	{
		return wiLocal.z;
	}

	Vector3 wiWorld()
	{
		return localFrame.localToWorld(wiLocal);
	}

	// Albedo methods, to calculate prob of different components
	Real albedoDiffuse(const Material& mat);
	Real albedoGlossy(const Material& mat);
	Real albedoReflect(const Material& mat);
	Real albedoTrans(const Material& mat);
	void calcComponentProb(const Material& mat , 
		ComponentProb& componentProb);
	
	// calculate BSDF value
	// pdf w.r.t solid angle
	// directPdf: sample wo given wi
	// reversePdf: sample wi given wo
	Color3 f(const Scene& scene , const Vector3& woWorld ,
		Real& cosWo , Real *directPdf = NULL , Real *reversePdf = NULL);
	Color3 calcDiffuse(const Material& mat , const Vector3& woLocal ,
		Real *directPdf = NULL , Real *reversePdf = NULL);
	Color3 calcGlossy(const Material& mat , const Vector3& woLocal ,
		Real *directPdf = NULL , Real *reversePdf = NULL);

	// calculate pdf
	// by default return directPdf
	Real pdf(const Scene& scene , const Vector3& woWorld ,
		const bool calcRevPdf = false);
	void pdfDiffuse(const Material& mat , const Vector3& woLocal ,
		Real *directPdf = NULL , Real *reversePdf = NULL);
	void pdfGlossy(const Material& mat , const Vector3& woLocal ,
		Real *directPdf = NULL , Real *reversePdf = NULL);

	// samples new direction from BSDF
	// return value is BSDF factor
	// also return sampled direction and pdf
	Color3 sample(const Scene& scene , const Vector3& rand3 ,
		Vector3& woWorld , Real& pdf , Real& cosWo , 
		int *sampledBSDFType = NULL);
	Color3 sampleDiffuse(const Material& mat , 
		const Vector3& rand3 , Vector3& woLocal , Real& pdf);
	Color3 sampleGlossy(const Material& mat , 
		const Vector3& rand3 , Vector3& woLocal , Real& pdf);
	Color3 sampleReflect(const Material& mat , 
		const Vector3& rand3 , Vector3& woLocal , Real& pdf);
	Color3 sampleTrans(const Material& mat , 
		const Vector3& rand3 , Vector3& woLocal , Real& pdf);
};

#endif