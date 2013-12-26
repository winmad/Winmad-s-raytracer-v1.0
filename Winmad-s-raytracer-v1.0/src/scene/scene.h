#ifndef SCENE_H
#define SCENE_H

#include "../geometry/ray.h"
#include "../geometry/geometry.h"
#include "../geometry/sphere.h"
#include "../geometry/triangle.h"
#include "../geometry/mesh.h"
#include "../geometry/intersection.h"
#include "../volume/volume.h"
#include "camera.h"
#include "light.h"
#include "KDtreeAccel.h"
#include "../parameters.h"
#include "../tinyobjloader/tiny_obj_loader.h"
#include <vector>
#include <map>

class Scene
{
public:
	std::vector<Geometry*> objs;
	KDtreeAccel kdtreeAccel;
	Camera camera;
	std::vector<Material> materials;
	std::vector<AbstractLight*> lights;
	SceneSphere sceneSphere;
	BackgroundLight *background;
	Volume *volume;
	Real totArea;

	Scene() : background(NULL) , totArea(0.f) {}

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
