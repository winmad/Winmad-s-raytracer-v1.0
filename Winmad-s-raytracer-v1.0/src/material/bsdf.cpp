#include "bsdf.h"
#include "fresnel.h"

Real BSDF::albedoDiffuse(const Material& mat)
{
	return luminance(mat.diffuse);
}

Real BSDF::albedoGlossy(const Material& mat)
{
	return luminance(mat.phong);
}

Real BSDF::albedoReflect(const Material& mat)
{
	return luminance(mat.specular);
}

Real BSDF::albedoTrans(const Material& mat)
{
	return 1.0f;
}

void BSDF::calcComponentProb(const Material& mat , 
	ComponentProb& componentProb)
{
	fresnelReflect = fresnelDielectric(wiLocal.z , mat.index);

	Real probDiffuse = albedoDiffuse(mat);
	Real probGlossy = albedoGlossy(mat);
	Real probReflect = fresnelReflect * albedoReflect(mat);
	Real probTrans = (1.f - fresnelReflect) * albedoTrans(mat);
	Real probTotal = probDiffuse + probGlossy + probReflect + probTrans;

	if (cmp(probTotal) <= 0)
	{
		componentProb.diffuseProb =
		componentProb.glossyProb =
		componentProb.reflectProb =
		componentProb.transProb = 
		continueProb = 0.f;
	}
	else
	{
		componentProb.diffuseProb = probDiffuse / probTotal;
		componentProb.glossyProb = probGlossy / probTotal;
		componentProb.reflectProb = probReflect / probTotal;
		componentProb.transProb = probTrans / probTotal;

		Color3 reflect = mat.diffuse + mat.phong +
			mat.specular * fresnelReflect;
		continueProb = reflect.maxComponent() + (1.f - fresnelReflect);
		continueProb = clampVal(continueProb , 0.f , 1.f);
	}
}

Color3 BSDF::calcDiffuse(const Material& mat , 
	const Vector3& woLocal , Real *directPdf /* = NULL  */, 
	Real *reversePdf /* = NULL */)
{
	if (cmp(componentProb.diffuseProb) == 0)
		return Color3(0);
	if (cmp(wiLocal.z) <= 0 || cmp(woLocal.z) <= 0)
		return Color3(0);
	if (directPdf)
		*directPdf += componentProb.diffuseProb *
			clampVal(woLocal.z * INV_PI , 0.0f , 1.0f);
	if (reversePdf)
		*reversePdf += componentProb.diffuseProb *
			clampVal(wiLocal.z * INV_PI , 0.0f , 1.0f);
	return mat.diffuse * INV_PI;
}

Color3 BSDF::calcGlossy(const Material& mat , 
	const Vector3& woLocal , Real *directPdf /* = NULL  */, 
	Real *reversePdf /* = NULL */) 
{
	if (cmp(componentProb.glossyProb) == 0)
		return Color3(0);
	if (cmp(wiLocal.z) <= 0 || cmp(woLocal.z) <= 0)
		return Color3(0);

	Vector3 reflectLocal = Vector3(-wiLocal.x , -wiLocal.y , wiLocal.z);
	Real cos = reflectLocal ^ woLocal;

	if (cmp(cos) == 0)
		return Color3(0);

	Real pdfW = componentProb.glossyProb *
		powerCosHemispherePdf(reflectLocal , woLocal , 
		mat.phongExp);

	if (directPdf)
		*directPdf += pdfW;
	if (reversePdf)
		*reversePdf += pdfW;

	Color3 rho = mat.phong * (mat.phongExp + 2.f) * 0.5f * INV_PI;
	return rho * std::pow(cos , mat.phongExp);
}

Color3 BSDF::f(const Scene& scene , const Vector3& woWorld , 
	Real& cosWo , Real *directPdf /* = NULL  */, 
	Real *reversePdf /* = NULL */)
{
	Color3 res(0.f);
	if (directPdf)
		*directPdf = 0.f;
	if (reversePdf)
		*reversePdf = 0.f;

	Vector3 woLocal = localFrame.worldToLocal(woWorld);
	if (cmp(woLocal.z * wiLocal.z) < 0)
		return res;

	cosWo = std::abs(woLocal.z);

	const Material& mat = scene.materials[matId];

	res = res + calcDiffuse(mat , woLocal , directPdf , reversePdf);
	res = res + calcGlossy(mat , woLocal , directPdf , reversePdf);
	return res;
}

void BSDF::pdfDiffuse(const Material& mat , const Vector3& woLocal , 
	Real *directPdf /* = NULL  */, Real *reversePdf /* = NULL */)
{
	if (cmp(componentProb.diffuseProb) == 0)
		return;
	
	if (directPdf)
		*directPdf += componentProb.diffuseProb *
		clampVal(woLocal.z , 0.f , 1.f) * INV_PI;

	if (reversePdf)
		*reversePdf += componentProb.diffuseProb *
		clampVal(wiLocal.z , 0.f , 1.f) * INV_PI;
}

void BSDF::pdfGlossy(const Material& mat , const Vector3& woLocal , 
	Real *directPdf /* = NULL  */, Real *reversePdf /* = NULL */)
{
	if (cmp(componentProb.glossyProb) == 0)
		return;

	Vector3 reflectLocal = Vector3(-wiLocal.x , -wiLocal.y , wiLocal.z);
	Real cos = reflectLocal ^ woLocal;

	if (cmp(cos) == 0)
		return;

	Real pdfW = componentProb.glossyProb *
		powerCosHemispherePdf(reflectLocal , woLocal , 
		mat.phongExp);

	if (directPdf)
		*directPdf += pdfW;
	if (reversePdf)
		*reversePdf += pdfW;
}

Real BSDF::pdf(const Scene& scene , 
	const Vector3& woWorld , const bool calcRevPdf /* = false */)
{
	Vector3 woLocal = localFrame.worldToLocal(woWorld);
	if (cmp(woLocal.z * wiLocal.z) < 0)
		return 0;

	const Material& mat = scene.materials[matId];

	Real directPdf = 0;
	Real reversePdf = 0;

	pdfDiffuse(mat , woLocal , &directPdf , &reversePdf);
	pdfGlossy(mat , woLocal , &directPdf , &reversePdf);

	return calcRevPdf ? reversePdf : directPdf;
}

Color3 BSDF::sampleDiffuse(const Material& mat , 
	const Vector3& rand3 , Vector3& woLocal , Real& pdf)
{
	if (cmp(wiLocal.z) <= 0)
		return Color3(0);

	Real pdfW;
	woLocal = sampleCosHemisphere(rand3 , &pdfW);
	pdf += pdfW * componentProb.diffuseProb;

	return mat.diffuse * INV_PI;
}

Color3 BSDF::sampleGlossy(const Material& mat , 
	const Vector3& rand3 , Vector3& woLocal , Real& pdf)
{
	woLocal = samplePowerCosHemisphere(rand3 , mat.phongExp , NULL);

	// numeric issues? need to recompute
	Vector3 reflectLocal = Vector3(-wiLocal.x , -wiLocal.y , wiLocal.z);
	Frame frame;
	frame.buildFromZ(reflectLocal);
	woLocal = frame.localToWorld(woLocal);

	Real cos = reflectLocal ^ woLocal;
	if (cmp(cos) <= 0)
		return Color3(0);

	pdfGlossy(mat , woLocal , &pdf);

	Color3 rho = mat.phong * (mat.phongExp + 2.f) * 0.5f * INV_PI;
	return rho * std::pow(cos , mat.phongExp);
}

Color3 BSDF::sampleReflect(const Material& mat , 
	const Vector3& rand3 , Vector3& woLocal , Real& pdf)
{
	woLocal = Vector3(-wiLocal.x , -wiLocal.y , wiLocal.z);
	pdf += componentProb.reflectProb;
	return mat.specular * fresnelReflect / std::abs(woLocal.z);
}

Color3 BSDF::sampleTrans(const Material& mat , 
	const Vector3& rand3 , Vector3& woLocal , Real& pdf)
{
	if (cmp(mat.index) < 0)
		return Color3(0);

	Real cosI = wiLocal.z;
	Real cosT , etaI_over_etaT;

	if (cmp(cosI) < 0) // hit from inside
	{
		etaI_over_etaT = mat.index;
		cosI = -cosI;
		cosT = 1.f;
	}
	else
	{
		etaI_over_etaT = 1.f / mat.index;
		cosT = -1.f;
	}

	Real sinI2 = 1.f - cosI * cosI;
	Real sinT2 = SQR(etaI_over_etaT) * sinI2;

	if (cmp(sinT2 - 1.f) < 0) // no total internal reflection
	{
		cosT *= std::sqrt(clampVal(1.f - sinT2 , 0.f , 1.f));
		woLocal = Vector3(-etaI_over_etaT * wiLocal.x ,
			-etaI_over_etaT * wiLocal.y , cosT);
		woLocal.normalize();
		pdf += componentProb.transProb;

		Real transCoeff = 1.f - fresnelReflect;

		return Color3(transCoeff / std::abs(cosT));
	}
	else
	{
		pdf += 0.f;
		return Color3(0);
	}
}

Color3 BSDF::sample(const Scene& scene , const Vector3& rand3 , 
	Vector3& woWorld , Real& pdf , Real& cosWo , 
	int *sampledBSDFType /* = NULL */)
{
	int sampledComponent;

	if (rand3.z < componentProb.diffuseProb)
		sampledComponent = BSDF_DIFFUSE;
	else if (rand3.z < componentProb.diffuseProb + componentProb.glossyProb)
		sampledComponent = BSDF_GLOSSY;
	else if (rand3.z < componentProb.diffuseProb + componentProb.glossyProb + componentProb.reflectProb)
		sampledComponent = BSDF_REFLECTION;
	else 
		sampledComponent = BSDF_TRANSMISSION;

	if (sampledBSDFType)
		*sampledBSDFType = sampledComponent;

	const Material& mat = scene.materials[matId];

	pdf = 0;
	Color3 res(0);
	Vector3 woLocal;

	if (sampledComponent == BSDF_DIFFUSE)
	{
		res = res + sampleDiffuse(mat , rand3 , woLocal , pdf);

		if (res.isBlack())
			return Color3(0);

		res = res + calcGlossy(mat , woLocal , &pdf);
	}
	else if (sampledComponent == BSDF_GLOSSY)
	{
		res = res + sampleGlossy(mat , rand3 , woLocal , pdf);

		if (res.isBlack())
			return Color3(0);

		res = res + calcDiffuse(mat , woLocal , &pdf);
	}
	else if (sampledComponent == BSDF_REFLECTION)
	{
		res = res + sampleReflect(mat , rand3 , woLocal , pdf);

		if (res.isBlack())
			return Color3(0);
	}
	else
	{
		res = res + sampleTrans(mat , rand3 , woLocal , pdf);

		if (res.isBlack())
			return Color3(0);
	}

	cosWo = std::abs(woLocal.z);
	if (cmp(cosWo) == 0)
		return Color3(0);

	woWorld = localFrame.localToWorld(woLocal);
	return res;
}