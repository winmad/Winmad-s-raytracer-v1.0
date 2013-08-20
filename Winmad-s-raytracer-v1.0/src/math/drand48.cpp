#include "drand48.h"

double drand48()
{
    drand48_seed = (drand48_a * drand48_seed + drand48_c) & 0xffffffffffffLL;
    unsigned int x = (drand48_seed >> 16);
    return ((double)x / (double)drand48_m);
}

void srand48(unsigned int i)
{
    drand48_seed = (((long long)i) << 16) | rand();
}
