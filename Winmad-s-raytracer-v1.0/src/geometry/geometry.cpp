#include "geometry.h"

Geometry::~Geometry() {}

void Geometry::setBox()
{
}

int Geometry::getMatId()
{
	return 0;
}

bool Geometry::hit(const Ray& ray , Intersection& inter) 
{
	return 0;
}

Real Geometry::getArea()
{
	return 0.f;
}

Vector3 Geometry::samplePos(const Vector3& samples , Vector3& normal)
{
	normal = Vector3(0.f);
	return Vector3(0.f);
}