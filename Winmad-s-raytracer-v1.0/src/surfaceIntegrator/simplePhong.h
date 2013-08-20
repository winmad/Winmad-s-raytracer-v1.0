#ifndef SIMPLE_PHONG_H
#define SIMPLE_PHONG_H

#include "../math/color.h"
#include "../math/vector.h"
#include "../geometry/geometry.h"

static const int specularCoefficient = 50;

Color3 getDiffuse(const Vector3& lightDir , const Vector3& normal ,
	const Color3& lightIntensity , const Color3& diffuse);

Color3 getSpecular(const Vector3& reflecDir , const Vector3& visionDir ,
	const Color3& lightIntensity , const Color3& specular);

Color3 getAmbient(const Color3& lightIntensity , const Color3& ambient);

Color3 getPhong(const Vector3& p , const Vector3& visionDir ,
	const Vector3& normal , const Vector3& lightSourcePos ,
	const Color3& lightIntensity , Geometry* obj , const Real& directCoe);

#endif