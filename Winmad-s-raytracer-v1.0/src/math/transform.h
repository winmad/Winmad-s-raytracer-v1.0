#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "math.h"
#include "vector.h"

struct Matrix4x4
{
	Real m[4][4];

	Matrix4x4()
	{
		m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.0f;
		m[0][1] = m[0][2] = m[0][3] = 0.0f;
		m[1][0] = m[1][2] = m[1][3] = 0.0f;
		m[2][0] = m[2][1] = m[2][3] = 0.0f;
		m[3][0] = m[3][1] = m[3][2] = 0.0f;
	}

	Matrix4x4(Real mat[4][4])
	{
		memcpy(m , mat , 16 * sizeof(Real));
	}

	Matrix4x4(Real t00 , Real t01 , Real t02 , Real t03 ,
			  Real t10 , Real t11 , Real t12 , Real t13 ,
			  Real t20 , Real t21 , Real t22 , Real t23 ,
			  Real t30 , Real t31 , Real t32 , Real t33)
	{
		m[0][0] = t00; m[0][1] = t01; m[0][2] = t02; m[0][3] = t03;
		m[1][0] = t10; m[1][1] = t11; m[1][2] = t12; m[1][3] = t13;
		m[2][0] = t20; m[2][1] = t21; m[2][2] = t22; m[2][3] = t23;
		m[3][0] = t30; m[3][1] = t31; m[3][2] = t32; m[3][3] = t33;
	}

	const Real* getPtr() const
	{
		return reinterpret_cast<const Real*>(this);
	}

	Real* getPtr()
	{
		return reinterpret_cast<Real*>(this);
	}

	void setRow(int r , Real a , Real b , Real c , Real d)
	{
		m[r][0] = a; m[r][1] = b; m[r][2] = c; m[r][3] = d;
	}

	void setRow(int r , const Vector3& a , Real b)
	{
		m[r][0] = a[0]; m[r][1] = a[1]; m[r][2] = a[2]; m[r][3] = b;
	}
};

bool operator ==(const Matrix4x4& m1 , const Matrix4x4& m2);
bool operator !=(const Matrix4x4& m1 , const Matrix4x4& m2);

const Matrix4x4 operator +(const Matrix4x4& m1 , const Matrix4x4& m2);
const Matrix4x4 operator *(const Matrix4x4& m1 , const Matrix4x4& m2);

Matrix4x4 transpose(const Matrix4x4 &m2);
Matrix4x4 inverse(const Matrix4x4& m2);

class Transform
{
public:
	Matrix4x4 m , mInv;

	Transform() {}

	Transform(Real mat[4][4])
	{
		m = Matrix4x4(mat);
		mInv = inverse(m);
	}

	Transform(const Matrix4x4& mat)
		: m(mat) , mInv(inverse(mat)) {}

	Transform(const Matrix4x4& mat , const Matrix4x4& matInv)
		: m(mat) , mInv(matInv) {}
};

Transform transpose(const Transform& t)
{
	return Transform(t.mInv , t.m);
}

Transform inverse(const Transform& t)
{
	return Transform(transpose(t.m) , transpose(t.mInv));
}

#endif