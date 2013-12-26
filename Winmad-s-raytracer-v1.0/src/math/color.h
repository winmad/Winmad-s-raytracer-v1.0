#ifndef COLOR_H
#define COLOR_H

#include "vector.h"

class Color3
{
public:
	Real r , g , b;

	Color3() 
		: r(0.0) , g(0.0) , b(0.0) {}

	Color3(Real val)
		: r(val) , g(val) , b(val) {}

	Color3(Real r , Real g , Real b)
		: r(r) , g(g) , b(b) {}

	Color3(const Color3& color)
	{
		*this = color;
	}

	Color3& operator =(const Color3& color)
	{
		r = color.r; g = color.g; b = color.b;
		return *this;
	}
	
	bool isBlack()
	{
		return (cmp(r) == 0.0) && (cmp(g) == 0.0)
			&& (cmp(b) == 0.0);
	}

	Real intensity()
	{
		return 0.2126f * r + 0.7152f * g + 0.0722f * b;
	}

	Real maxComponent()
	{
		return std::max(r , std::max(g , b));
	}

	void clamp()
	{
		r = clampVal(r , 0.0 , 1.0);
		g = clampVal(g , 0.0 , 1.0);
		b = clampVal(b , 0.0 , 1.0);
	}
	
	void gamma(Real _gamma)
	{
		Real invGamma = 1.f / _gamma;
		r = std::pow(r , invGamma);
		g = std::pow(g , invGamma);
		b = std::pow(b , invGamma);
	}

	unsigned char R()
	{
		return (unsigned char)(r * 255.0);
	}

	unsigned char G()
	{
		return (unsigned char)(g * 255.0);
	}

	unsigned char B()
	{
		return (unsigned char)(b * 255.0);
	}
};

const Color3 operator +(const Color3& , const Color3&);
const Color3 operator -(const Color3& , const Color3&);
const Color3 operator *(const Color3& , const Real&);
const Color3 operator /(const Color3& , const Real&);

const Color3 operator -(const Color3&);

// It is not dot product!
const Color3 operator |(const Color3& , const Color3&);

const Color3 exp(const Color3&);

Real luminance(const Color3& c);

void printColor3(FILE *fp , const Color3& color);

#endif
