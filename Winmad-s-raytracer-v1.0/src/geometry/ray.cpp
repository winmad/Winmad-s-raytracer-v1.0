#include "ray.h"

Real calcT(const Vector3& st , const Vector3& ed , const Vector3& dir)
{
	if (cmp(dir.x) != 0)
		return (ed.x - st.x) / dir.x;
	else if (cmp(dir.y) != 0)
		return (ed.y - st.y) / dir.y;
	else if (cmp(dir.z) != 0)
		return (ed.z - st.z) / dir.z;
	return INF;
}