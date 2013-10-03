#ifndef SCENE_H
#define SCENE_H

#include "../geometry/ray.h"
#include "../geometry/geometry.h"
#include "../geometry/sphere.h"
#include "../geometry/triangle.h"
#include "../geometry/mesh.h"
#include "../geometry/intersection.h"
#include "camera.h"
#include "light.h"
#include "KDtree.h"
#include "../parameters.h"
#include <vector>

class ViewPort
{
public:
	Vector3 l , r;
	Vector3 delta;

	ViewPort() {}

	ViewPort(Real xmin , Real xmax , Real ymin , Real ymax , 
		Real zmin , Real zmax) 
		: l(xmin , ymin , zmin) , r(xmax , ymax , zmax) {}
};

class Scene
{
public:
	int pointLightNum;
	
	std::vector<PointLight> lightlist;
    std::vector<Triangle> areaLightlist;
	std::vector<Geometry*> objlist;
	KDtree kdtree;
	Vector3 camera;
	ViewPort viewPort;

	Scene() {}

	void addGeometry(Geometry *g);

	void addLight(PointLight l);

	void loadScene();

	void loadScene(char *filename);

	void init(char* filename , Parameters& para);

    Geometry* intersect(const Ray& ray , Intersection& inter);

    bool intersect(const Ray& ray);

	Real shadowRayTest(const Ray& ray , const Vector3& p);
};

#endif
