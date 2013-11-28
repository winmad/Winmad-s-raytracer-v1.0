#ifndef CAMERA_H
#define CAMERA_H

#include "../math/vector.h"
#include "../math/transform.h"
#include "../geometry/ray.h"

class Camera
{
public:
	Vector3 pos;
	Vector3 forward;
	Vector3 up;
	Transform worldToCamera;
	Transform cameraToWorld;
	Transform rasterToWorld;
	Transform worldToRaster;
	Real imagePlaneDist , xResolution , yResolution , horizontalFOV;

	void setup(const Vector3& _pos , const Vector3& _forward ,
		const Vector3& _up , Real _xResolution , Real _yResolution ,
		Real _horizontalFOV);
	
	bool checkRaster(Real x , Real y);

	Ray generateRay(Real x , Real y);
};

#endif