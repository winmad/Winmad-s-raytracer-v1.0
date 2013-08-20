#ifndef MATERIAL_H
#define MATERIAL_H

#include "../math/color.h"
#include "../math/vector.h"
#include "bxdf.h"
#include "phong.h"

class Material
{
public:
	BxDF *bxdf;

	Real shininess;
	Real transparency;
	Real refractionIndex;

	Material() 
	{
		bxdf = new Phong();

		shininess = 0.0;
		transparency = 0.0;
		refractionIndex = 0.0;
	}

	void setMaterial(const Color3& _kd , const Color3& _ks ,
		const Real& _shininess , const Real& _transparency , 
		const Real& _refractionIndex)
	{
		bxdf->kd = _kd;
		bxdf->ks = _ks;
		shininess = _shininess;
		transparency = _transparency;
		refractionIndex = _refractionIndex;
	}
};

#endif
