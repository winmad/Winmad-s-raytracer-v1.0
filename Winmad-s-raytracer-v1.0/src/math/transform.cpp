#include "transform.h"

// Matrix class
bool operator ==(const Matrix4x4& m1 , const Matrix4x4& m2) 
{
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			if (cmp(m1.m[i][j] - m2.m[i][j]) != 0)
				return 0;
	return 1;
}

bool operator !=(const Matrix4x4 &m1 , const Matrix4x4& m2) 
{
	return !(m1 == m2);
}

const Matrix4x4 operator +(const Matrix4x4& m1 , const Matrix4x4& m2)
{
	Matrix4x4 res;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			res.m[i][j] = m1.m[i][j] + m2.m[i][j];
	return res;
}

const Matrix4x4 operator *(const Matrix4x4& m1 , const Matrix4x4& m2)
{
	Matrix4x4 res;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			res.m[i][j] = m1.m[i][0] * m2.m[0][j] +
						  m1.m[i][1] * m2.m[1][j] +
						  m1.m[i][2] * m2.m[2][j] + 
						  m1.m[i][3] * m2.m[3][j];
	return res;
}

Matrix4x4 transpose(const Matrix4x4 &m2)
{
	Matrix4x4 res;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			res.m[i][j] = m2.m[j][i];
	return res;
}

Matrix4x4 inverse(const Matrix4x4& m2)
{
	const Real *m = m2.getPtr();
	Real inv[16], det;
	int i;

	inv[0] = m[5] * m[10] * m[15] -
		m[5]  * m[11] * m[14] -
		m[9]  * m[6]  * m[15] +
		m[9]  * m[7]  * m[14] +
		m[13] * m[6]  * m[11] -
		m[13] * m[7]  * m[10];

	inv[4] = -m[4]  * m[10] * m[15] +
		m[4]  * m[11] * m[14] +
		m[8]  * m[6]  * m[15] -
		m[8]  * m[7]  * m[14] -
		m[12] * m[6]  * m[11] +
		m[12] * m[7]  * m[10];

	inv[8] = m[4]  * m[9] * m[15] -
		m[4]  * m[11] * m[13] -
		m[8]  * m[5] * m[15] +
		m[8]  * m[7] * m[13] +
		m[12] * m[5] * m[11] -
		m[12] * m[7] * m[9];

	inv[12] = -m[4]  * m[9] * m[14] +
		m[4]  * m[10] * m[13] +
		m[8]  * m[5] * m[14] -
		m[8]  * m[6] * m[13] -
		m[12] * m[5] * m[10] +
		m[12] * m[6] * m[9];

	inv[1] = -m[1]  * m[10] * m[15] +
		m[1]  * m[11] * m[14] +
		m[9]  * m[2] * m[15] -
		m[9]  * m[3] * m[14] -
		m[13] * m[2] * m[11] +
		m[13] * m[3] * m[10];

	inv[5] = m[0]  * m[10] * m[15] -
		m[0]  * m[11] * m[14] -
		m[8]  * m[2] * m[15] +
		m[8]  * m[3] * m[14] +
		m[12] * m[2] * m[11] -
		m[12] * m[3] * m[10];

	inv[9] = -m[0]  * m[9] * m[15] +
		m[0]  * m[11] * m[13] +
		m[8]  * m[1] * m[15] -
		m[8]  * m[3] * m[13] -
		m[12] * m[1] * m[11] +
		m[12] * m[3] * m[9];

	inv[13] = m[0]  * m[9] * m[14] -
		m[0]  * m[10] * m[13] -
		m[8]  * m[1] * m[14] +
		m[8]  * m[2] * m[13] +
		m[12] * m[1] * m[10] -
		m[12] * m[2] * m[9];

	inv[2] = m[1]  * m[6] * m[15] -
		m[1]  * m[7] * m[14] -
		m[5]  * m[2] * m[15] +
		m[5]  * m[3] * m[14] +
		m[13] * m[2] * m[7] -
		m[13] * m[3] * m[6];

	inv[6] = -m[0]  * m[6] * m[15] +
		m[0]  * m[7] * m[14] +
		m[4]  * m[2] * m[15] -
		m[4]  * m[3] * m[14] -
		m[12] * m[2] * m[7] +
		m[12] * m[3] * m[6];

	inv[10] = m[0]  * m[5] * m[15] -
		m[0]  * m[7] * m[13] -
		m[4]  * m[1] * m[15] +
		m[4]  * m[3] * m[13] +
		m[12] * m[1] * m[7] -
		m[12] * m[3] * m[5];

	inv[14] = -m[0]  * m[5] * m[14] +
		m[0]  * m[6] * m[13] +
		m[4]  * m[1] * m[14] -
		m[4]  * m[2] * m[13] -
		m[12] * m[1] * m[6] +
		m[12] * m[2] * m[5];

	inv[3] = -m[1] * m[6] * m[11] +
		m[1] * m[7] * m[10] +
		m[5] * m[2] * m[11] -
		m[5] * m[3] * m[10] -
		m[9] * m[2] * m[7] +
		m[9] * m[3] * m[6];

	inv[7] = m[0] * m[6] * m[11] -
		m[0] * m[7] * m[10] -
		m[4] * m[2] * m[11] +
		m[4] * m[3] * m[10] +
		m[8] * m[2] * m[7] -
		m[8] * m[3] * m[6];

	inv[11] = -m[0] * m[5] * m[11] +
		m[0] * m[7] * m[9] +
		m[4] * m[1] * m[11] -
		m[4] * m[3] * m[9] -
		m[8] * m[1] * m[7] +
		m[8] * m[3] * m[5];

	inv[15] = m[0] * m[5] * m[10] -
		m[0] * m[6] * m[9] -
		m[4] * m[1] * m[10] +
		m[4] * m[2] * m[9] +
		m[8] * m[1] * m[6] -
		m[8] * m[2] * m[5];

	det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

	Matrix4x4 res;

	if (det == 0)
		res;

	det = 1.f / det;

	for (i = 0; i < 16; i++)
		res.getPtr()[i] = inv[i] * det;

	return res;
}

// Transform class
bool operator ==(const Transform& t1 , const Transform& t2)
{
	return t1.m == t2.m && t1.mInv == t2.mInv;
}

bool operator !=(const Transform& t1 , const Transform& t2)
{
	return !(t1 == t2);
}

bool operator <(const Transform& t1 , const Transform& t2)
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			if (t1.m.m[i][j] < t2.m.m[i][j]) return 1;
			if (t1.m.m[i][j] > t2.m.m[i][j]) return 0;
		}
	}
	return 0;
}

const Transform operator *(const Transform& t1 , const Transform& t2)
{
	Matrix4x4 m = t1.m * t2.m;
	Matrix4x4 mInv = t2.mInv * t1.mInv;
	return Transform(m , mInv);
}

bool Transform::isIndentity()
{
	return (cmp(m.m[0][0] - 1.0f) == 0 &&
			cmp(m.m[1][1] - 1.0f) == 0 &&
			cmp(m.m[2][2] - 1.0f) == 0 &&
			cmp(m.m[3][3] - 1.0f) == 0 &&
			cmp(m.m[0][1]) == 0 && cmp(m.m[0][2]) == 0 &&
			cmp(m.m[0][3]) == 0 && cmp(m.m[1][0]) == 0 &&
			cmp(m.m[1][2]) == 0 && cmp(m.m[1][3]) == 0 &&
			cmp(m.m[2][0]) == 0 && cmp(m.m[2][1]) == 0 &&
			cmp(m.m[2][3]) == 0 && cmp(m.m[3][0]) == 0 &&
			cmp(m.m[3][1]) == 0 && cmp(m.m[3][2]) == 0);
}

bool Transform::hasScale()
{
	Real lx2 = tVector(Vector3(1.0f , 0.0f , 0.0f)).sqrLength();
	Real ly2 = tVector(Vector3(0.0f , 1.0f , 0.0f)).sqrLength();
	Real lz2 = tVector(Vector3(0.0f , 0.0f , 1.0f)).sqrLength();
	return (cmp(lx2 - 1.0f) != 0 ||
			cmp(ly2 - 1.0f) != 0 ||
			cmp(lz2 - 1.0f) != 0);
}

Transform inverse(const Transform& t)
{
	return Transform(t.mInv , t.m);
}

Transform transpose(const Transform& t)
{
	return Transform(transpose(t.m) , transpose(t.mInv));
}

void Transform::print(FILE *fp)
{
	fprintf(fp , "-------------transform------------\n");
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
			fprintf(fp , "%.3lf " , m.m[i][j]);
		fprintf(fp , "\n");
	}
}

// Basic Transforms
Transform identity()
{
	return Transform();
}

Transform translate(const Vector3& delta)
{
	Matrix4x4 m(1 , 0 , 0 , delta.x ,
				0 , 1 , 0 , delta.y ,
				0 , 0 , 1 , delta.z ,
				0 , 0 , 0 , 1);
	Matrix4x4 mInv(1 , 0 , 0 , -delta.x ,
				   0 , 1 , 0 , -delta.y ,
				   0 , 0 , 1 , -delta.z ,
				   0 , 0 , 0 , 1);
	return Transform(m , mInv);
}

Transform scale(Real x , Real y , Real z)
{
	Matrix4x4 m(x , 0 , 0 , 0 ,
				0 , y , 0 , 0 ,
				0 , 0 , z , 0 ,
				0 , 0 , 0 , 1);
	Matrix4x4 mInv(1.0f / x , 0 , 0 , 0 ,
				   0 , 1.0f / y , 0 , 0 ,
				   0 , 0 , 1.0f / z , 0 ,
				   0 , 0 , 0 , 1);
	return Transform(m , mInv);
}

Transform rotateX(Real angle)
{
	Real sinT = sinf(angle / 180.0f * PI);
	Real cosT = cosf(angle / 180.0f * PI);
	Matrix4x4 m(1 , 0 , 0 , 0 ,
				0 , cosT , -sinT , 0 ,
				0 , sinT , cosT , 0 ,
				0 , 0 , 0 , 1);
	return Transform(m , transpose(m));
}

Transform rotateY(Real angle)
{
	Real sinT = sinf(angle / 180.0f * PI);
	Real cosT = cosf(angle / 180.0f * PI);
	Matrix4x4 m(cosT , 0 , sinT , 0 ,
				0 , 1 , 0 , 0 ,
				-sinT , 0 , cosT , 0 ,
				0 , 0 , 0 , 1);
	return Transform(m , transpose(m));
}

Transform rotateZ(Real angle)
{
	Real sinT = sinf(angle / 180.0f * PI);
	Real cosT = cosf(angle / 180.0f * PI);
	Matrix4x4 m(cosT , -sinT , 0 , 0 ,
				sinT , cosT , 0 , 0 ,
				0 , 0 , 1 , 0 ,
				0 , 0 , 0 , 1);
	return Transform(m , transpose(m));
}

Transform rotate(Real angle , const Vector3& axis)
{
	Vector3 a = axis;
	a.normalize();
	Real sinT = sinf(angle / 180.0f * PI);
	Real cosT = cosf(angle / 180.0f * PI);

	Real m[4][4];
	m[0][0] = a.x * a.x + (1.0f - a.x * a.x) * cosT;
	m[0][1] = a.x * a.y * (1.0f - cosT) - a.z * sinT;
	m[0][2] = a.x * a.z * (1.0f - cosT) + a.y * sinT;
	m[0][3] = 0;

	m[1][0] = a.x * a.y * (1.0f - cosT) + a.z * sinT;
	m[1][1] = a.y * a.y + (1.0f - a.y * a.y) * cosT;
	m[1][2] = a.y * a.z * (1.0f - cosT) - a.x * sinT;	
	m[1][3] = 0;

	m[2][0] = a.x * a.z * (1.0f - cosT) - a.y * sinT;
	m[2][1] = a.y * a.z * (1.0f - cosT) + a.x * sinT;
	m[2][2] = a.z * a.z + (1.0f - a.z * a.z) * cosT;
	m[2][3] = 0;

	m[3][0] = m[3][1] = m[3][2] = 0;
	m[3][3] = 1;

	Matrix4x4 mat(m);
	return Transform(mat , transpose(mat));
}

// _up -> x axis, _left -> y axis, -_dir -> z axis
Transform lookAt(const Vector3& pos , const Vector3& look , 
	const Vector3& up)
{
	Vector3 _dir = look - pos;
	_dir.normalize();
	Vector3 _up = up * (-_dir);
	_up.normalize();
	Vector3 _left = _up * _dir;

	Matrix4x4 worldToCamera;
	Vector3 p(_up ^ pos , _left ^ pos , -_dir ^ pos);

	worldToCamera.setRow(0 , _up , -p.x);
	worldToCamera.setRow(1 , _left , -p.y);
	worldToCamera.setRow(2 , -_dir , -p.z);

	return Transform(worldToCamera , inverse(worldToCamera));
}

Transform orthographic(Real znear , Real zfar)
{
	return scale(1.0f , 1.0f , 1.0f / (zfar - znear)) *
		translate(Vector3(0.0f , 0.0f , -znear));
}

// camera points towards -z
Transform perspective(Real fov , Real znear , Real zfar)
{
	Matrix4x4 persp = Matrix4x4(1 , 0 , 0 , 0 ,
								0 , -1 , 0 , 0 ,
								0 , 0 , (znear + zfar) / (zfar - znear) , 2 * zfar * znear / (zfar - znear) ,
								0 , 0 , -1 , 0);
	Real invTanAng = 1.0f / tanf(fov / 360.0f * PI);
	return scale(invTanAng , invTanAng , 1) * Transform(persp);
}

