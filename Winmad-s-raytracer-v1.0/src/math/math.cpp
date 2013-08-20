#include "math.h"

Real clampVal(Real val , Real minVal , Real maxVal)
{
	return std::min(maxVal , std::max(val , minVal));
}

int cmp(const Real& x)
{
	return ((x < -eps) ? -1 : (x > eps));
}
