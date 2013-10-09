#include "triangle.h"

int Triangle::getMatId()
{
	return matId;
}

bool Triangle::hit(const Ray& ray , Intersection& inter)
{
	Real A = p0.x - p1.x;
	Real B = p0.y - p1.y;
	Real C = p0.z - p1.z;

	Real D = p0.x - p2.x;
	Real E = p0.y - p2.y;
	Real F = p0.z - p2.z;

	Real G = ray.dir.x;
	Real H = ray.dir.y;
	Real I = ray.dir.z;

	Real J = p0.x - ray.origin.x;
	Real K = p0.y - ray.origin.y;
	Real L = p0.z - ray.origin.z;

	Real EIHF = E * I - H * F;
	Real GFDI = G * F - D * I;
	Real DHEG = D * H - E * G;

	Real denom = (A * EIHF + B * GFDI + C * DHEG);

	Real beta = (J * EIHF + K * GFDI + L * DHEG) / denom;
	if (cmp(beta) < 0 || cmp(beta - 1) > 0)
	{
		inter.t = INF;
		return 0;
	}

	Real AKJB = A * K - J * B;
	Real JCAL = J * C - A * L;
	Real BLKC = B * L - K * C;

	Real gamma = (I * AKJB + H * JCAL + G * BLKC) / denom;
	if (cmp(gamma) < 0 || cmp(beta + gamma - 1) > 0)
	{
		inter.t = INF;
		return 0;
	}

	inter.t = -(F * AKJB + E * JCAL + D * BLKC) / denom;
	if (cmp(inter.t) <= 0)
	{
		inter.t = INF;
		return 0;
	}

	if (cmp(inter.t - ray.tmin) < 0 ||
		cmp(inter.t - ray.tmax) > 0)
	{
		inter.t = INF;
		return 0;
	}

	inter.p = ray(inter.t);
	inter.n = (p1 - p0) * (p2 - p0);
	inter.n.normalize();
	if (cmp(ray.dir ^ inter.n) < 0)
		inter.inside = 0;
	else 
		inter.inside = 1;
	inter.matId = matId;
	return 1;
}