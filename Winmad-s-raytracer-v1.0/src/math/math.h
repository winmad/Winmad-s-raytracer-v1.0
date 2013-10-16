#ifndef MATH_H
#define MATH_H

#include <cstdio>
#include <cmath>
#include <algorithm>
#include <cassert>

#define SQR(x) ((x) * (x))

typedef float Real;

const Real PI = acos(-1.0f);
const Real INV_PI = 1.0f / PI;
const Real EPS = 1e-3f;
const Real INF = 1e10f;

Real clampVal(Real val , Real minVal , Real maxVal);

int cmp(const Real& x);

Real pdfWtoA(const Real pdfW , const Real dist , const Real cos);

Real pdfAtoW(const Real pdfA , const Real dist , const Real cos);

#endif