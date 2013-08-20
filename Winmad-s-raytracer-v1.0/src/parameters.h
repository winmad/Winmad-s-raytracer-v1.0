#ifndef PARAMETERS_H
#define PARAMETERS_H

class Parameters
{
public:
    int MAX_TRACING_DEPTH;

    int SAMPLES_PER_PIXEL;

// direct illumination
    int SAMPLES_OF_LIGHT;

// indirect illumination
    int SAMPLES_OF_HEMISPHERE;

    int HEIGHT;

    int WIDTH;

    int PHONG_POWER_INDEX;

    int POINT_LIGHT_NUM;

    void load_parameters(char *filename);
};

#endif
