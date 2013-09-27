#ifndef RNG_H
#define RNG_H

#include <cstdlib>
#include <stdint.h>
#include "vector.h"

class RNG
{
public:
	static const int N = 624;
	mutable unsigned long mt[N];
	mutable int mti;
	
	RNG(uint32_t _seed = 5489UL)
	{
		mti = N + 1;
		seed(_seed);
	}

	void seed(uint32_t _seed) const;
	float randFloat() const;
	uint32_t randUInt() const;
	Vector3 randVector3() const;
};

#endif
