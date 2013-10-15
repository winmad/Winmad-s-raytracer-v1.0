#include "film.h"
#include <opencv2/opencv.hpp>

void ImageFilm::addColor(int h , int w , const Color3& c)
{
	color[h][w] = color[h][w] + c;
}

void ImageFilm::scale(Real coeff)
{
	for (int i = 0; i < height; i++)
		for (int j = 0 ; j < width; j++)
			color[i][j] = color[i][j] * coeff;	
}

void ImageFilm::clamp()
{
	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++)
			color[i][j].clamp();
}

void ImageFilm::gamma(Real _gamma)
{
	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++)
			color[i][j].gamma(_gamma);
}

void ImageFilm::outputImage(char *filename , Real _scale , Real _gamma)
{
	IplImage *img = 0;
	img = cvCreateImage(cvSize(width , height) , 
		IPL_DEPTH_8U , 3);

	scale(_scale);
	clamp();
	gamma(_gamma);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int h = img->height;
			int w = img->width;
			int step = img->widthStep;
			int channels = img->nChannels;

			uchar *data = (uchar*)img->imageData;
			data[i * step + j * channels + 0] = color[i][j].B();
			data[i * step + j * channels + 1] = color[i][j].G();
			data[i * step + j * channels + 2] = color[i][j].R();
		}
	}
	cvSaveImage(filename , img);
}