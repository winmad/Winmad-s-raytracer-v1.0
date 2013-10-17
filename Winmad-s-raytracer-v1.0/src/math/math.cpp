#include "math.h"

Real clampVal(Real val , Real minVal , Real maxVal)
{
	return std::min(maxVal , std::max(val , minVal));
}

int cmp(const Real x)
{
	return ((x < -EPS) ? -1 : (x > EPS));
}

Real pdfWtoA(const Real pdfW , const Real dist , const Real cos)
{
	return pdfW * std::abs(cos) / SQR(dist);
}

Real pdfAtoW(const Real pdfA , const Real dist , const Real cos)
{
	return pdfA * SQR(dist) / std::abs(cos);
}