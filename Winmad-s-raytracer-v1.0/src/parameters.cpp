#include "parameters.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

static FILE* fp;
static char str[1024];

int read_int()
{
    int res;
    while (fscanf(fp , "%s" , str) != EOF)
    {
        if (strlen(str) <= 0 || str[0] == '#' || str[0] == '\n')
            continue;
        res = atoi(str);
        return res;
    }
    return -1;
}

void Parameters::load_parameters(char *filename)
{
    fp = fopen(filename , "r");
    MAX_TRACING_DEPTH = read_int();
    SAMPLES_PER_PIXEL = read_int();
    SAMPLES_OF_LIGHT = read_int();
    SAMPLES_OF_HEMISPHERE = read_int();
    WIDTH = read_int();
    HEIGHT = read_int();
    PHONG_POWER_INDEX = read_int();
    POINT_LIGHT_NUM = read_int();
    fclose(fp);
}
