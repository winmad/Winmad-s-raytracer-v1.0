#ifndef LIGHT_H
#define LIGHT_H

#include "../math/color.h"
#include "../math/vector.h"

class PointLight
{
public:
	Vector3 pos;
	Color3 color;

	PointLight() {}

	PointLight(Vector3 pos , Color3 color)
		: pos(pos) , color(color) {}
};

#endif