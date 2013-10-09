#include "light.h"
#include "../sampler/sampler.h"

Color3 AreaLight::illuminance(const SceneSphere& sceneSphere , 
	const Vector3& pos , const Vector3& rand3 , 
	Vector3& dirToLight , Real& dist , Real& directPdf , 
	Real *emissionPdf /* = NULL  */, Real *cosAtLight /* = NULL */)
{
	Vector3 lightPoint = sampleTriangle(rand3 , p0 , p0 + d1 , p0 + d2);
	dirToLight = lightPoint - pos;
	dist = dirToLight.length();
	dirToLight = dirToLight / dist;

	Real cosNormalDir = localFrame.normal() ^ (-dirToLight);

	if (cmp(cosNormalDir) <= 0)
		return Color3(0);

	directPdf = invArea * SQR(dist) / cosNormalDir;

	if (cosAtLight)
		*cosAtLight = cosNormalDir;

	if (emissionPdf)
		*emissionPdf = invArea * cosNormalDir * INV_PI;

	return intensity;
}

Color3 AreaLight::emit(const SceneSphere& sceneSphere , 
	const Vector3& dirRand3 , const Vector3& posRand3 , 
	Vector3& pos , Vector3& dir , Real& emissionPdf , 
	Real *directPdfArea , Real *cosAtLight)
{
	pos = sampleTriangle(posRand3 ,p0 , p0 + d1 , p0 + d2);
	Vector3 localDirOut = sampleCosHemisphere(dirRand3 , &emissionPdf);
	emissionPdf *= invArea;

	localDirOut.z = std::max(localDirOut.z , EPS);
	dir = localFrame.localToWorld(localDirOut);

	if (directPdfArea)
		*directPdfArea = invArea;
	
	if (cosAtLight)
		*cosAtLight = localDirOut.z;

	return intensity * localDirOut.z;
}

Color3 AreaLight::getRadiance(const SceneSphere& sceneSphere , 
	const Vector3& rayDir , const Vector3& hitPos , 
	Real *directPdfArea /* = NULL  */, Real *emissionPdf /* = NULL */)
{
	Real cosNormalDir = clampVal(localFrame.normal() ^ (-rayDir) , 0.f , 1.f);

	if (cmp(cosNormalDir) == 0)
		return Color3(0);

	if (directPdfArea)
		*directPdfArea = invArea;

	if (emissionPdf)
	{
		*emissionPdf = cosHemispherePdf(localFrame.normal() , -rayDir);
		*emissionPdf *= invArea;
	}

	return intensity;
}

bool AreaLight::isFinite()
{
	return 1;
}

bool AreaLight::isDelta()
{
	return 0;
}

Color3 DirectionalLight::illuminance(const SceneSphere& sceneSphere , 
	const Vector3& pos , const Vector3& rand3 , 
	Vector3& dirToLight , Real& dist , Real& directPdf , 
	Real *emissionPdf /* = NULL  */, Real *cosAtLight /* = NULL */)
{
	dirToLight = -localFrame.normal();
	dist = INF;
	directPdf = 1.f;

	if (emissionPdf)
		*emissionPdf = uniformDiskPdf() * sceneSphere.invSceneRadiusSqr;

	if (cosAtLight)
		*cosAtLight = 1.f;

	return intensity;
}

Color3 DirectionalLight::emit(const SceneSphere& sceneSphere ,
	const Vector3& dirRand3 , const Vector3& posRand3 , 
	Vector3& pos , Vector3& dir , Real& emissionPdf , 
	Real *directPdfArea , Real *cosAtLight)
{
	Vector3 xy = sampleUniformDisk(posRand3);

	pos = sceneSphere.sceneCenter + (-localFrame.normal() +
		localFrame.binormal() * xy.x + localFrame.tangent() * xy.y) *
		sceneSphere.sceneRadius;
	dir = localFrame.normal();

	emissionPdf = uniformDiskPdf() * sceneSphere.invSceneRadiusSqr;

	if (directPdfArea)
		*directPdfArea = 1.f;

	if (cosAtLight)
		*cosAtLight = 1.f;

	return intensity;
}

Color3 DirectionalLight::getRadiance(const SceneSphere& sceneSphere ,
	const Vector3& rayDir , const Vector3& hitPos , 
	Real *directPdfArea /* = NULL  */, Real *emissionPdf /* = NULL */)
{
	return Color3(0);
}

bool DirectionalLight::isFinite()
{
	return 0;
}

bool DirectionalLight::isDelta()
{
	return 1;
}

Color3 PointLight::illuminance(const SceneSphere& sceneSphere , 
	const Vector3& pos , const Vector3& rand3 , 
	Vector3& dirToLight , Real& dist , Real& directPdf , 
	Real *emissionPdf /* = NULL  */, Real *cosAtLight /* = NULL */)
{
	dirToLight = lightPos - pos;
	dist = dirToLight.length();
	dirToLight = dirToLight / dist;

	directPdf = SQR(dist);

	if (emissionPdf)
		*emissionPdf = uniformSpherePdf();

	if (cosAtLight)
		*cosAtLight = 1.f;

	return intensity;
}

Color3 PointLight::emit(const SceneSphere& sceneSphere , 
	const Vector3& dirRand3 , const Vector3& posRand3 , 
	Vector3& pos , Vector3& dir , Real& emissionPdf , 
	Real *directPdfArea , Real *cosAtLight)
{
	pos = lightPos;
	dir = sampleUniformSphere(dirRand3 , &emissionPdf);

	if (directPdfArea)
		*directPdfArea = 1.f;

	if (cosAtLight)
		*cosAtLight = 1.f;

	return intensity;
}

Color3 PointLight::getRadiance(const SceneSphere& sceneSphere , 
	const Vector3& rayDir , const Vector3& hitPos , 
	Real *directPdfArea /* = NULL  */, Real *emissionPdf /* = NULL */)
{
	return Color3(0);
}

bool PointLight::isFinite()
{
	return 1;
}

bool PointLight::isDelta()
{
	return 1;
}

Color3 BackgroundLight::illuminance(const SceneSphere& sceneSphere , 
	const Vector3& pos , const Vector3& rand3 , Vector3& dirToLight , 
	Real& dist , Real& directPdf , 
	Real *emissionPdf /* = NULL  */, Real *cosAtLight /* = NULL */)
{
	dirToLight = sampleUniformSphere(rand3 , &directPdf);

	dist = INF;

	if (emissionPdf)
		*emissionPdf = directPdf * uniformDiskPdf() *
			sceneSphere.invSceneRadiusSqr;

	if (cosAtLight)
		*cosAtLight = 1.f;

	return backgroundColor * scale;
}

Color3 BackgroundLight::emit(const SceneSphere& sceneSphere , 
	const Vector3& dirRand3 , const Vector3& posRand3 , 
	Vector3& pos , Vector3& dir , Real& emissionPdf , 
	Real *directPdfArea , Real *cosAtLight)
{
	Real directPdf;

	dir = sampleUniformSphere(dirRand3 , &directPdf);

	Vector3 xy = sampleUniformDisk(posRand3);
	Frame frame;
	frame.buildFromZ(dir);

	pos = sceneSphere.sceneCenter + (-frame.normal() + 
		frame.binormal() * xy.x + frame.tangent() * xy.y) *
		sceneSphere.sceneRadius;

	emissionPdf = directPdf * uniformDiskPdf() *
		sceneSphere.invSceneRadiusSqr;

	if (directPdfArea)
		*directPdfArea = directPdf;

	if (cosAtLight)
		*cosAtLight = 1.f;

	return backgroundColor * scale;
}

Color3 BackgroundLight::getRadiance(const SceneSphere& sceneSphere , 
	const Vector3& rayDir , const Vector3& hitPos , 
	Real *directPdfArea /* = NULL  */, Real *emissionPdf /* = NULL */)
{
	Real directPdf = uniformSpherePdf();

	Real positionPdf = uniformDiskPdf() * sceneSphere.invSceneRadiusSqr;

	if (directPdfArea)
		*directPdfArea = directPdf;

	if (emissionPdf)
		*emissionPdf = directPdf * positionPdf;

	return backgroundColor * scale;
}

bool BackgroundLight::isFinite()
{
	return 0;
}

bool BackgroundLight::isDelta()
{
	return 0;
}
