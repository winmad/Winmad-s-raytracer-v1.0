#include "vector.h"

const Vector3 operator +(const Vector3& left , const Vector3& right)
{
	return Vector3(left.x + right.x , left.y + right.y , left.z + right.z);
}

const Vector3 operator -(const Vector3& left , const Vector3& right)
{
	return Vector3(left.x - right.x , left.y - right.y , left.z - right.z);
}

const Real operator ^(const Vector3& left , const Vector3& right)
{
	return left.x * right.x + left.y * right.y + left.z * right.z;
}

const Vector3 operator |(const Vector3& left , const Vector3& right)
{
	return Vector3(left.x * right.x , left.y * right.y , left.z * right.z);
}

const Vector3 operator *(const Vector3& left , const Vector3& right)
{
	return Vector3(left.y * right.z - left.z * right.y ,
				   left.z * right.x - left.x * right.z ,
				   left.x * right.y - left.y * right.x);
}

const Vector3 operator *(const Vector3& left , const Real& right)
{
	return Vector3(left.x * right , left.y * right , left.z * right);
}

const Vector3 operator /(const Vector3& left , const Real& right)
{
	if (cmp(right) == 0) return Vector3(inf , inf , inf);
	return Vector3(left.x / right , left.y / right , left.z / right);
}

bool operator ==(const Vector3& left , const Vector3& right)
{
	return (cmp(left.x - right.x) == 0 && cmp(left.y - right.y) == 0 &&
			cmp(left.z - right.z) == 0);
}

bool operator !=(const Vector3& left , const Vector3& right)
{
	return !(left == right);
}

Vector3 getReflectDir(const Vector3& wi , const Vector3& n)
{
	Vector3 res = n * (n ^ wi) * 2.0 - wi;
	res.normalize();
	return res;
}

Vector3 getTransDir(const Vector3& wi , const Vector3& n , 
	const Real& refractionIndex , int inside)
{
	Real refraction;
	if (!inside) 
		refraction = refractionIndex;
	else
		refraction = 1.0f / refractionIndex;
	Vector3 N = n * (inside == 0 ? 1.0f : -1.0f);
	Real cosI = (N ^ wi);
	Real cosT = 1.0f - SQR(refraction) * (1.0f - SQR(cosI));
	Vector3 res;
	if (cmp(cosT) > 0)
	{
		res = -wi * refraction + N * (refraction * cosI - sqrt(cosT));
		res.normalize();
		return res;
	}
	else
	{
		return Vector3(inf , inf , inf);
	}
}

void printVector3(FILE *fp , const Vector3& v)
{
	fprintf(fp , "(%.3lf,%.3lf,%.3lf)" , v.x , v.y , v.z);
}