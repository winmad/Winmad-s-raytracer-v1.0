#ifndef INTERSECTION_H
#define INTERSECTION_H

#include "../math/vector.h"

class Intersection
{
public:
	Real t;
	Vector3 p , n;
	int inside;
	int matId;

	Intersection() {}

	Intersection(Real _t , Vector3 _p , Vector3 _n , 
		int _inside) 
		: t(_t) , p(_p) , n(_n) , inside(_inside) {}
};

#endif