#include "rng.h"

#define M 397
#define MATRIX_A 0x9908b0dfUL
#define UPPER_MASK 0x80000000UL
#define LOWER_MASK 0x7fffffffUL

void RNG::seed(uint32_t _seed) const
{
	mt[0] = _seed & 0xffffffffUL;
	for (mti = 1; mti < N; mti++)
	{
		mt[mti] = (1812433253UL * (mt[mti - 1] ^ (mt[mti - 1] >> 30)) + mti);
		mt[mti] &= 0xffffffffUL;
	}
}

float RNG::randFloat() const
{
	float v = (randUInt() & 0xffffff) / float(1 << 24);
	return v;
}

uint32_t RNG::randUInt() const
{
	unsigned long y;
	static unsigned long mag01[2] = {0x0UL , MATRIX_A};

	if (mti >= N)
	{
		int k;
		if (mti == N + 1)
		{
			seed(5489UL);
		}

		for (k = 0; k < N - M; k++)
		{
			y = (mt[k] & UPPER_MASK) | (mt[k + 1] & LOWER_MASK);
			mt[k] = mt[k + M] ^ (y >> 1) ^ mag01[y & 0x1UL];
		}

		for (; k < N - 1; k++)
		{
			y = (mt[k] & UPPER_MASK) | (mt[k + 1] & LOWER_MASK);
			mt[k] = mt[k + (M - N)] ^ (y >> 1) ^ mag01[y & 0x1UL];
		}

		y = (mt[N - 1] & UPPER_MASK) | (mt[0] & LOWER_MASK);
		mt[N - 1] = mt[M - 1] ^ (y >> 1) ^ mag01[y & 0x1UL];

		mti = 0;
	}
	y = mt[mti++];

	y ^= (y >> 11);
	y ^= (y << 7) & 0x9d2c5680UL;
	y ^= (y << 15) & 0xefc60000UL;
	y ^= (y >> 18);

	return y;
}

Vector3 RNG::randVector3() const
{
	Real a = randFloat();
	Real b = randFloat();
	Real c = randFloat();
	return Vector3(a , b , c);
}