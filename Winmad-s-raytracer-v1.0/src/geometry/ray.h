#ifndef RAY_H
#define RAY_H

#include "../math/vector.h"

class Ray
{
public:
	Vector3 origin , dir;

	Ray() {}
	Ray(Vector3 origin , Vector3 dir)
		: origin(origin) , dir(dir) {this->dir.normalize();}

	Vector3 operator ()(Real t) const
	{
		return origin + dir * t;
	}

	Ray(const Ray& ray) 
	{
		*this = ray;
	}
};

Real calcT(const Vector3& st , const Vector3& ed , const Vector3& dir);

#endif