#include "fresnel.h"

Real fresnelDielectric(Real cosI , Real index)
{
	if (cmp(index) < 0)
		return 1.0f;

	Real etaI_over_etaT;

	if (cmp(cosI) < 0)
	{
		cosI = -cosI;
		etaI_over_etaT = index;
	}
	else
	{
		etaI_over_etaT = 1.0f / index;
	}

	Real sinT_sqr = SQR(etaI_over_etaT) * (1.0f - SQR(cosI));
	Real cosT = std::sqrt(std::max(0.0f , 1.0f - sinT_sqr));

	Real term1 = etaI_over_etaT * cosT;
	Real parallel = (cosI - term1) / (cosI + term1);

	Real term2 = etaI_over_etaT * cosI;
	Real perpendicular = (term2 - cosT) / (term2 + cosT);

	return 0.5f * (SQR(parallel) + SQR(perpendicular));
}