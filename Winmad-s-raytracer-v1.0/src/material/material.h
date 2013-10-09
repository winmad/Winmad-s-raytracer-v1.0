#ifndef MATERIAL_H
#define MATERIAL_H

#include "../math/color.h"
#include "../math/vector.h"

class Material
{
public:
	Color3 diffuse;

	Color3 phong;
	Real phongExp;

	Color3 specular;
	Real index; // index of refraction

	Material() 
	{
		init();
	}

	void init()
	{
		diffuse = Color3(0);
		phong = Color3(0);
		phongExp = 1;
		specular = Color3(0);
		index = -1;
	}
};

#endif
