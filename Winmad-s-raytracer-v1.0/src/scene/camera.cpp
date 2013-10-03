#include "camera.h"

void Camera::setup(const Vector3& _pos , const Vector3& _forward , const Vector3& _up , 
	Real _xResolution , Real _yResolution , Real _horizontalFOV)
{
	pos = _pos;
	forward = _forward;
	forward.normalize();
	up = _up;
	up.normalize();

	xResolution = _xResolution;
	yResolution = _yResolution;

	worldToCamera = lookAt(pos , pos + forward , up);
	cameraToWorld = Transform(worldToCamera.mInv , worldToCamera.m);

	Transform persp = perspective(_horizontalFOV , 0.1f , 10000.f);
	Transform worldToNScreen = persp * worldToCamera;
	Transform nScreenToWorld = inverse(worldToNScreen);

	worldToRaster = scale(xResolution * 0.5f , yResolution * 0.5f , 0) *
		translate(Vector3(1.0f , 1.0f , 0.0f)) * worldToNScreen;
	rasterToWorld = nScreenToWorld * translate(Vector3(-1.0f , -1.0f , 0.0f)) *
		scale(2.0f / xResolution , 2.0f / yResolution , 0);

	Real tanHalfAngle = tanf(_horizontalFOV * PI / 360.0f);
	imagePlaneDist = xResolution / (2.0f * tanHalfAngle);
}

bool Camera::checkRaster(Real x , Real y)
{
	return (cmp(x) >= 0 && cmp(y) >= 0 && 
		cmp(x - xResolution) < 0 && cmp(y - yResolution) < 0);
}

Ray Camera::generateRay(Real x , Real y)
{
	Vector3 p = rasterToWorld.tPoint(Vector3(x , y , 0));
	Ray res(pos , p - pos);
	return res;
}