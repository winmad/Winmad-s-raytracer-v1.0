#ifndef FRAME_H
#define FRAME_H

#include "vector.h"

class Frame
{
public:
	Vector3 x , y , z;

	Frame()
	{
		x = Vector3(1.0f , 0.0f , 0.0f);
		y = Vector3(0.0f , 1.0f , 0.0f);
		z = Vector3(0.0f , 0.0f , 1.0f);
	}

	Frame(const Vector3& _x , const Vector3& _y , const Vector3& _z)
		: x(_x) , y(_y) , z(_z) {}

	void buildFromZ(const Vector3& _z);

	Vector3 localToWorld(const Vector3& l);

	Vector3 worldToLocal(const Vector3& w);

	const Vector3& binormal() const
	{
		return x;
	}

	const Vector3 tangent() const
	{
		return y;
	}

	const Vector3 normal() const
	{
		return z;
	}
};

#endif