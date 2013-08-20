#ifndef DRAND48_H
#define DRAND48_H

#include <cstdlib>

const long long drand48_m = 0x100000000LL;
const long long drand48_c = 0xb16;
const long long drand48_a = 0x5deece66dLL;

static unsigned long long drand48_seed = 1;

double drand48();

void srand48(unsigned int i);

#endif
