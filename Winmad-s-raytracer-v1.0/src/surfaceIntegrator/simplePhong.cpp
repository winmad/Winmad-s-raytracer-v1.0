#include "simplePhong.h"
#include "../scene/light.h"

Color3 getDiffuse(const Vector3& lightDir , const Vector3& normal , 
	const Color3& lightIntensity , const Color3& diffuse)
{
	Real coe = std::max(0.0 , lightDir ^ normal);
	Color3 res = (lightIntensity | diffuse) * coe;
	res.clamp();
	return res;
}

Color3 getSpecular(const Vector3& reflecDir , const Vector3& visionDir , 
	const Color3& lightIntensity , const Color3& specular)
{
	Real coe = std::max(0.0 , reflecDir ^ visionDir);
	coe = pow(coe , specularCoefficient);
	Color3 res = (lightIntensity | specular) * coe;
	res.clamp();
	return res;
}

Color3 getAmbient(const Color3& lightIntensity , const Color3& ambient)
{
	Color3 res = (lightIntensity | ambient);
	res.clamp();
	return res;
}

Color3 getPhong(const Vector3& p , const Vector3& visionDir ,
	const Vector3& normal , const Vector3& lightSourcePos ,
	const Color3& lightIntensity , Geometry* obj , const Real& directCoe)
{
	Vector3 lightDir = lightSourcePos - p;
	lightDir.normalize();
	Vector3 reflecDir = normal * ((normal ^ lightDir) * 2.0) - lightDir;
	reflecDir.normalize();
	Color3 diffuseIntensity = getDiffuse(lightDir , normal , 
		lightIntensity , obj->getMaterial().bxdf->kd);
	Color3 specularIntensity = getSpecular(reflecDir , visionDir , 
		lightIntensity , obj->getMaterial().bxdf->ks);
	Color3 res = (diffuseIntensity + specularIntensity) * directCoe;
	res.clamp();
	return res;
}