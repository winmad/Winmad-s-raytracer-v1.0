#include "surfaceIntegrator.h"
#include <opencv2/opencv.hpp>

void SurfaceIntegrator::init(char *filename , Parameters& para)
{
}

Color3 SurfaceIntegrator::raytracing(const Ray& ray , int dep)
{
	return Color3(0.0 , 0.0 , 0.0);
}

void SurfaceIntegrator::render(char *filename)
{
	IplImage *img = 0;
	img = cvCreateImage(cvSize(width , height) , 
		IPL_DEPTH_8U , 3);
	Color3 res;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
            res = Color3(0.0 , 0.0 , 0.0);

            for (int k = 0; k < samplesPerPixel; k++)
            {
                Vector3 v0 = Vector3(j - 0.5 , i - 0.5 , 0);
                Vector3 v1 = Vector3(j + 0.5 , i - 0.5 , 0);
				Vector3 v2 = Vector3(j - 0.5 , i + 0.5 , 0);
                
                Vector3 posRaster = sampleRectangleStratified(
                    rng.randVector3() ,
					v0 , v1 , v2 , k , samplesPerPixel);
                
                Ray ray = scene.camera.generateRay(posRaster.x , posRaster.y);

                Color3 tmp = raytracing(ray , 0);
                tmp.clamp();
                res = res + tmp;
            }
            res = res * (1.0 / (Real)samplesPerPixel);
            /*
            fprintf(fp , "c=(%.3lf,%.3lf,%.3lf)\n" ,
                    res.r , res.g , res.b);
            */
			int h = img->height;
			int w = img->width;
			int step = img->widthStep;
			int channels = img->nChannels;

			uchar *data = (uchar*)img->imageData;
			data[i * step + j * channels + 0] = res.B();
			data[i * step + j * channels + 1] = res.G();
			data[i * step + j * channels + 2] = res.R();
		}
	}
	cvSaveImage(filename , img);
}
