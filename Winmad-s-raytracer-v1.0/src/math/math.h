#ifndef MATH_H
#define MATH_H

#include <cstdio>
#include <cmath>
#include <algorithm>

#define SQR(x) ((x) * (x))

typedef double Real;

const Real PI = acos(-1.0f);
const Real eps = 1e-3f;
const Real inf = 1e10f;

Real clampVal(Real val , Real minVal , Real maxVal);

int cmp(const Real& x);

#endif