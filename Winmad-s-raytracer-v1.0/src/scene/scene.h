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
#include <map>

class Scene
{
public:
	std::vector<Geometry*> objs;
	KDtree kdtree;
	Camera camera;
	std::vector<Material> materials;
	std::vector<AbstractLight*> lights;
	SceneSphere sceneSphere;
	BackgroundLight *background;

	Scene() : background(NULL) {}

	~Scene()
	{
		delete background;
		for (int i = 0; i < objs.size(); i++)
			delete objs[i];
		for (int i = 0; i < lights.size(); i++)
			delete lights[i];
	}

	Geometry* intersect(const Ray& ray , Intersection& inter);

	bool intersect(const Ray& ray);

	Real shadowRayTest(const Ray& ray , const Vector3& p);

	bool occluded(const Vector3& p1 , const Vector3& dir , const Vector3& p2);

	void addGeometry(Geometry *g);

	void addLight(AbstractLight *l);

	void addMaterial(Material mat);

	void loadScene();

	void loadScene(char *filename);

	void init(char* filename , Parameters& para);
};

#endif
