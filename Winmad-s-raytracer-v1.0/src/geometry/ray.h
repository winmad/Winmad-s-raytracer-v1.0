#ifndef RAY_H
#define RAY_H

#include "../math/vector.h"

class Ray
{
public:
	Vector3 origin , dir;
	Real tmin , tmax;

	Ray() : tmin(0.0f) , tmax(INF) {}

	Ray(const Vector3& origin , const Vector3& dir)
		: origin(origin) , dir(dir) ,
		tmin(0.0f) , tmax(INF) {this->dir.normalize();}

	Ray(const Vector3& origin , const Vector3& dir , 
		Real tmin , Real tmax)
		: origin(origin) , dir(dir) , 
		tmin(tmin) , tmax(tmax) {this->dir.normalize();}

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