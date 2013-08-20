#include "color.h"

const Color3 operator +(const Color3& left , const Color3& right)
{
	Color3 res = Color3(left.r + right.r , left.g + right.g , left.b + right.b);
	return res;
}

const Color3 operator -(const Color3& left , const Color3& right)
{
	Color3 res = Color3(left.r - right.r , left.g - right.g , left.b - right.b);
	return res;
}

const Color3 operator *(const Color3& left , const Real& right)
{
	Color3 res = Color3(left.r * right , left.g * right , left.b * right);
	return res;
}

const Color3 operator |(const Color3& left , const Color3& right)
{
	Color3 res = Color3(left.r * right.r , left.g * right.g , left.b * right.b);
	return res;
}

const Color3 operator /(const Color3& left , const Real& right)
{
	Color3 res = Color3(left.r / right , left.g / right , left.b / right);
	return res;
}

void printColor3(FILE *fp , const Color3& color)
{
	fprintf(fp , "(%.3lf,%.3lf,%.3lf)" , color.r , color.g , color.b);
}
