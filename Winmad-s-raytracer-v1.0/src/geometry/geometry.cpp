#include "geometry.h"

Geometry::~Geometry() {}

void Geometry::setBox()
{
}

Material& Geometry::getMaterial()
{
	Material material;
	return material;
}

bool Geometry::hit(const Ray& ray , Intersection& inter) 
{
	return 0;
}
