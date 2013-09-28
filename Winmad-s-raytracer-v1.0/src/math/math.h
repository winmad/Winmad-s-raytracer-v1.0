#ifndef MATH_H
#define MATH_H

#include <cstdio>
#include <cmath>
#include <algorithm>
#include <assert.h>
#include <crtdbg.h>

#define SQR(x) ((x) * (x))

typedef double Real;

const Real PI = acos(-1.0f);
const Real INV_PI = 1.0f / PI;
const Real EPS = 1e-3f;
const Real INF = 1e10f;

Real clampVal(Real val , Real minVal , Real maxVal);

int cmp(const Real& x);

#endif