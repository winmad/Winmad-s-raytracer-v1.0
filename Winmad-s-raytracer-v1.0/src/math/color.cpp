#include "color.h"

const Color3 operator -(const Color3& c)
{
	Color3 res = Color3(-c.r , -c.g , -c.b);
	return res;
}

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

const Color3 exp(const Color3& c)
{
	return Color3(std::exp(c.r) , std::exp(c.g) , std::exp(c.b));
}

Real luminance(const Color3& c)
{
	return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b;
}

void printColor3(FILE *fp , const Color3& color)
{
	fprintf(fp , "(%.3lf,%.3lf,%.3lf)\n" , color.r , color.g , color.b);
}
