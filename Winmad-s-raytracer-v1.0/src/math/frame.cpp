#include "frame.h"

void Frame::buildFromZ(const Vector3& _z)
{
	z = _z;
	z.normalize();
	Vector3 tmpX = (std::abs(z.x) > 0.99f) ? Vector3(0.0f , 1.0f , 0.0f) : Vector3(1.0f , 0.0f , 0.0f);
	y = z * tmpX;
	y.normalize();
	x = y * z;
}

Vector3 Frame::localToWorld(const Vector3& l)
{
	return x * l.x + y * l.y + z * l.z;
}

Vector3 Frame::worldToLocal(const Vector3& w)
{
	return Vector3(w ^ x , w ^ y , w ^ z);
}