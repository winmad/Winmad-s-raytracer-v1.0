#ifndef LIGHT_H
#define LIGHT_H

#include "../math/color.h"
#include "../math/vector.h"
#include "../math/frame.h"

struct SceneSphere
{
	Vector3 sceneCenter;
	Real sceneRadius;
	Real invSceneRadiusSqr;
};

class AbstractLight
{
public:
	/* Illuminates a given point in the scene.
     *
     * Given a point and two random samples (e.g., for position on area lights),
     * this method returns direction from point to light, distance,
     * pdf of having chosen this direction (e.g., 1 / area).
     * Optionally also returns pdf of emitting particle in this direction,
     * and cosine from lights normal (helps with PDF of hitting the light,
     * but set to 1 for point lights).
     *
     * Returns radiance.
     */
	virtual Color3 illuminance(const SceneSphere& sceneSphere ,
		const Vector3& pos , const Vector3& rand3 , 
		Vector3& dirToLight , Real& dist , Real& directPdf ,
		Real *emissionPdf = NULL , Real *cosAtLight = NULL) = 0;

	/* Emits particle from the light.
     *
     * Given two sets of random numbers (e.g., position and direction on area light),
     * this method generates a position and direction for light particle, along
     * with the pdf.
     *
     * Can also supply pdf (w.r.t. area) of choosing this position when calling
     * Illuminate. Also provides cosine on the light (this is 1 for point lights etc.).
     *
     * Returns "energy" that particle carries
     */
	virtual Color3 emit(const SceneSphere& sceneSphere ,
		const Vector3& dirRand3 , const Vector3& posRand3 ,
		Vector3& pos , Vector3& dir , Real& emissionPdf ,
		Real *directPdfArea , Real *cosAtLight) = 0;

	/* Returns radiance for ray randomly hitting the light
     *
     * Given ray direction and hitpoint, it returns radiance.
     * Can also provide area pdf of sampling hitpoint in Illuminate,
     * and of emitting particle along the ray (in opposite direction).
     */
	virtual Color3 getRadiance(const SceneSphere& sceneSphere ,
		const Vector3& rayDir , const Vector3& hitPos ,
		Real *directPdfArea = NULL , Real *emissionPdf = NULL) = 0;

	virtual bool isFinite() = 0;

	virtual bool isDelta() = 0;
};

class AreaLight : public AbstractLight
{
public:
	Vector3 p0 , d1 , d2;
	Frame localFrame;
	Color3 intensity;
	Real invArea;

	AreaLight(const Vector3& _p0 , const Vector3& _p1 ,
		const Vector3& _p2 , const Color3& _intensity)
		: intensity(_intensity)
	{
		p0 = _p0;
		d1 = _p1 - _p0;
		d2 = _p2 - _p0;

		Vector3 n = d1 * d2;
		Real len = n.length();
		invArea = 2.f / len;
		localFrame.buildFromZ(n);
	}

	virtual Color3 illuminance(const SceneSphere& sceneSphere ,
		const Vector3& pos , const Vector3& rand3 , 
		Vector3& dirToLight , Real& dist , Real& directPdf ,
		Real *emissionPdf = NULL , Real *cosAtLight = NULL);

	virtual Color3 emit(const SceneSphere& sceneSphere ,
		const Vector3& dirRand3 , const Vector3& posRand3 ,
		Vector3& pos , Vector3& dir , Real& emissionPdf ,
		Real *directPdfArea , Real *cosAtLight);

	virtual Color3 getRadiance(const SceneSphere& sceneSphere ,
		const Vector3& rayDir , const Vector3& hitPos ,
		Real *directPdfArea = NULL , Real *emissionPdf = NULL);

	virtual bool isFinite();

	virtual bool isDelta();
};

class DirectionalLight : public AbstractLight
{
public:
	Frame localFrame;
	Color3 intensity;

	DirectionalLight(const Vector3& _dir , const Color3& _intensity)
		: intensity(_intensity)
	{
		localFrame.buildFromZ(_dir);
	}

	virtual Color3 illuminance(const SceneSphere& sceneSphere ,
		const Vector3& pos , const Vector3& rand3 , 
		Vector3& dirToLight , Real& dist , Real& directPdf ,
		Real *emissionPdf = NULL , Real *cosAtLight = NULL);

	virtual Color3 emit(const SceneSphere& sceneSphere ,
		const Vector3& dirRand3 , const Vector3& posRand3 ,
		Vector3& pos , Vector3& dir , Real& emissionPdf ,
		Real *directPdfArea , Real *cosAtLight);

	virtual Color3 getRadiance(const SceneSphere& sceneSphere ,
		const Vector3& rayDir , const Vector3& hitPos ,
		Real *directPdfArea = NULL , Real *emissionPdf = NULL);

	virtual bool isFinite();

	virtual bool isDelta();
};

class PointLight : public AbstractLight
{
public:
	Vector3 lightPos;
	Color3 intensity;

	PointLight(const Vector3& _pos , const Color3& _intensity)
		: lightPos(_pos) , intensity(_intensity) {}

	virtual Color3 illuminance(const SceneSphere& sceneSphere ,
		const Vector3& pos , const Vector3& rand3 , 
		Vector3& dirToLight , Real& dist , Real& directPdf ,
		Real *emissionPdf = NULL , Real *cosAtLight = NULL);

	virtual Color3 emit(const SceneSphere& sceneSphere ,
		const Vector3& dirRand3 , const Vector3& posRand3 ,
		Vector3& pos , Vector3& dir , Real& emissionPdf ,
		Real *directPdfArea , Real *cosAtLight);

	virtual Color3 getRadiance(const SceneSphere& sceneSphere ,
		const Vector3& rayDir , const Vector3& hitPos ,
		Real *directPdfArea = NULL , Real *emissionPdf = NULL);

	virtual bool isFinite();

	virtual bool isDelta();
};

class BackgroundLight : public AbstractLight
{
public:
	Color3 backgroundColor;
	Real scale;

	BackgroundLight()
	{
		backgroundColor = Color3(135 , 206 , 250) / 255.f;
		scale = 1.f;
	}

	virtual Color3 illuminance(const SceneSphere& sceneSphere ,
		const Vector3& pos , const Vector3& rand3 , 
		Vector3& dirToLight , Real& dist , Real& directPdf ,
		Real *emissionPdf = NULL , Real *cosAtLight = NULL);

	virtual Color3 emit(const SceneSphere& sceneSphere ,
		const Vector3& dirRand3 , const Vector3& posRand3 ,
		Vector3& pos , Vector3& dir , Real& emissionPdf ,
		Real *directPdfArea , Real *cosAtLight);

	virtual Color3 getRadiance(const SceneSphere& sceneSphere ,
		const Vector3& rayDir , const Vector3& hitPos ,
		Real *directPdfArea = NULL , Real *emissionPdf = NULL);

	virtual bool isFinite();

	virtual bool isDelta();
};

#endif