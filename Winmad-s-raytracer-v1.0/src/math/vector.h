#ifndef VECTOR_H
#define VECTOR_H

#include "math.h"

class Vector3
{
public:
	Real x , y , z;

	Vector3()
		: x(0) , y(0) , z(0) {}

	Vector3(Real x , Real y , Real z)
		: x(x) , y(y) , z(z) {}

	Vector3(const Vector3& v)
	{
		*this = v;
	}

	Vector3& operator =(const Vector3& v)
	{
		x = v.x; y = v.y; z = v.z;
		return *this;
	}

	Real operator [](int i) const
	{
		if (i < 0 || i > 2) 
			return inf;
		if (i == 0) return x;
		if (i == 1) return y;
		if (i == 2) return z;
	}

	Real& operator [](int i)
	{
		if (i == 0) return x;
		if (i == 1) return y;
		if (i == 2) return z;
	}

	const Vector3 operator -() const
	{
		return Vector3(-x , -y , -z);
	}

	Real sqrLength()
	{
		return SQR(x) + SQR(y) + SQR(z);
	}

	void normalize()
	{
		Real len = sqrt(this->sqrLength());
		x /= len; y /= len; z /= len;
	}

	bool isNormal()
	{
		return cmp(sqrLength() - 1.0f) == 0;
	}
};

const Vector3 operator +(const Vector3& , const Vector3&);
const Vector3 operator -(const Vector3& , const Vector3&);
const Real operator ^(const Vector3& , const Vector3&);
const Vector3 operator |(const Vector3& , const Vector3&);
const Vector3 operator *(const Vector3& , const Real&);
const Vector3 operator *(const Vector3& , const Vector3&);
const Vector3 operator /(const Vector3& , const Real&);
bool operator ==(const Vector3& , const Vector3&);
bool operator !=(const Vector3& , const Vector3&);


Vector3 getReflectDir(const Vector3& wi , const Vector3& n);
Vector3 getTransDir(const Vector3& wi , const Vector3& n ,
	const Real& refractionIndex , int inside);

void printVector3(FILE *fp , const Vector3& v);

#endif