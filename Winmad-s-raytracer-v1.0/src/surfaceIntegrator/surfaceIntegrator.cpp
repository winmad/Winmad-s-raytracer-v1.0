#include "surfaceIntegrator.h"

void SurfaceIntegrator::init(char *filename , Parameters& para)
{
}

Color3 SurfaceIntegrator::raytracing(const Ray& ray , int dep)
{
	return Color3(0.0 , 0.0 , 0.0);
}

//static FILE *fp = fopen("debug_integrator.txt" , "w");

void SurfaceIntegrator::render()
{
	Color3 res;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
            res = Color3(0.f);

            for (int k = 0; k < samplesPerPixel; k++)
            {
				
                Vector3 v0 = Vector3(j - 0.5f , i - 0.5f , 0);
                Vector3 v1 = Vector3(j + 0.5f , i - 0.5f , 0);
				Vector3 v2 = Vector3(j - 0.5f , i + 0.5f , 0);
                
                Vector3 posRaster = sampleRectangleStratified(
                    rng.randVector3() ,
					v0 , v1 , v2 , k , samplesPerPixel);

                Ray ray = scene.camera.generateRay(posRaster.x , posRaster.y);

                Color3 tmp = raytracing(ray , 0);

// 				fprintf(fp , "c(%d,%d)=(%.3lf,%.3lf,%.3lf)\n" , i , j ,
// 					tmp.r , tmp.g , tmp.b);

				film->addColor(i , j , tmp);
            }
		}
	}
	film->scale(1.f / samplesPerPixel);
}

void SurfaceIntegrator::outputImage(char *filename)
{
	film->outputImage(filename , 1.f , 2.2f);
}