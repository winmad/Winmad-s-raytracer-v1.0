#ifndef FILM_H
#define FILM_H

#include "../math/color.h"

class ImageFilm
{
public:
	int height , width;

	Color3 **color;

	ImageFilm() : height(0) , width(0) , color(NULL) {}

	ImageFilm(int height , int width)
		: height(height) , width(width)
	{
		color = new Color3*[height];
		for (int i = 0; i < height; i++)
			color[i] = new Color3[width];

		for (int i = 0; i < height; i++)
			for (int j = 0; j < width; j++)
				color[i][j] = Color3(0.f);
	}

	~ImageFilm()
	{
		for (int i = 0; i < height; i++)
			delete[] color[i];
		delete[] color;
	}

	void addColor(int h , int w , const Color3& c);

	void scale(Real coeff);

	void clamp();

	void gamma(Real _gamma);

	void outputImage(char *filename , Real _scale , Real _gamma);
};

#endif