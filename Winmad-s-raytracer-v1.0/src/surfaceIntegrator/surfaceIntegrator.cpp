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
	Real y = viewPort.l.y;
	for (int i = 0; i < height; i++)
	{
		Real x = viewPort.l.x;
		for (int j = 0; j < width; j++)
		{
            res = Color3(0.0 , 0.0 , 0.0);
            for (int k = 0; k < samplesPerPixel; k++)
            {
                Vector3 v0 = Vector3(x - 0.5 * viewPort.delta.x ,
                                     y - 0.5 * viewPort.delta.y ,
                                     viewPort.l.z);
                
                Vector3 v1 = Vector3(x + 0.5 * viewPort.delta.x ,
                                     y - 0.5 * viewPort.delta.y ,
                                     viewPort.l.z);
                
                Vector3 v2 = Vector3(x - 0.5 * viewPort.delta.x ,
                                     y + 0.5 * viewPort.delta.y ,
                                     viewPort.l.z);
                
                Vector3 pos_on_viewport = sampleOnRectangleStratified(
                    v0 , v1 , v2 , k , samplesPerPixel);
                
                Ray ray = Ray(scene.camera , 
                              pos_on_viewport - scene.camera);

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
			data[(h - i - 1) * step + j * channels + 0] = res.B();
			data[(h - i - 1) * step + j * channels + 1] = res.G();
			data[(h - i - 1) * step + j * channels + 2] = res.R();

			x += viewPort.delta.x;
		}
		y += viewPort.delta.y;
	}
	cvSaveImage(filename , img);
}

