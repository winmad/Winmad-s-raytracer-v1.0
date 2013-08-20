#include "scene.h"
#include "../sampler/sampler.h"
#include "../tinyxml/tinyxml.h"

void Scene::addGeometry(Geometry* g)
{
	objlist.push_back(g);
}

void Scene::addLight(PointLight l)
{
	lightlist.push_back(l);
}

Geometry* Scene::intersect(const Ray& ray , Intersection& inter)
{
    Geometry *g = NULL;
    g = kdtree.traverse(ray , kdtree.root);
    if (g != NULL)
        g->hit(ray , inter);
    return g;
}

bool Scene::intersect(const Ray& ray)
{
    Geometry *g = NULL;
    g = kdtree.traverse(ray , kdtree.root);
    if (g == NULL)
        return 0;
    else
        return 1;
}

Real Scene::shadowRayTest(const Ray& ray , const Vector3& p)
{
	Geometry *g = kdtree.traverse(ray , kdtree.root);
	Intersection inter;

	if (g == NULL)
		return 1.0;

	g->hit(ray , inter);

	if (ray(inter.t) == p)
		return 1.0;
	else
		return 0.0;
}

// build-in scene: many balls
void Scene::loadScene()
{
	PointLight l = PointLight(Vector3(-3 , 5 , 1) ,
		Color3(1 , 1 , 1));
	addLight(l);

 	Sphere *s;
 	s = new Sphere(Vector3(2 , 0.8 , 3.0) , 2);
 	s->material.setMaterial(Color3(0.7 , 0.7 , 1.0) , 
 		Color3(0 , 0 , 0) , 0.2 , 0.8 , 0.7);
 	addGeometry(s);
 
 	s = new Sphere(Vector3(-5.5 , -0.5 , 7) , 2);
 	s->material.setMaterial(Color3(0.7 , 0.7 , 1.0) , 
 		Color3(0.1 , 0.1 , 0.1) , 0.5 , 0.5 , 0.7);
 	addGeometry(s);
 
 	for (int x = 0; x < 10; x++)
 	{
 		for (int y = 0; y < 10; y++)
 		{
 			s = new Sphere(Vector3(-2.0 + x * 1.2 ,
 				-4.3 + y * 1.5 , 10 + x * 0.3) , 0.3);
 			s->material.setMaterial(Color3(0.3 , 1.0 , 0.4) ,
 				Color3(0.6 , 0.6 , 0.6) , 0.0 , 0.5 , 0.0);
 			addGeometry(s);
 		}
 	}

	camera = Vector3(0 , 0 , -5);

    viewPort.l = Vector3(-4 , -3 , 0);
    viewPort.r = Vector3(4 , 3 , 0);
}

// load .scene, xml format
void Scene::loadScene(char* filename)
{
	TiXmlDocument doc = TiXmlDocument(filename);
	doc.LoadFile();

	TiXmlElement *root = doc.RootElement();
	
	TiXmlElement *it = root->FirstChildElement();

	while (it)
	{
		if (it->ValueStr() == "camera")
		{
			TiXmlElement *attr = it->FirstChildElement();
			attr->Attribute("x" , &camera.x);
			attr->Attribute("y" , &camera.y);
			attr->Attribute("z" , &camera.z);
			
		}
		else if (it->ValueStr() == "viewport")
		{
			TiXmlElement *attr = it->FirstChildElement();
			attr->Attribute("x1" , &viewPort.l.x);
			attr->Attribute("x2" , &viewPort.r.x);
			attr->Attribute("y1" , &viewPort.l.y);
			attr->Attribute("y2" , &viewPort.r.y);
			attr->Attribute("z1" , &viewPort.l.z);
			attr->Attribute("z2" , &viewPort.r.z);
		}
		else if (it->ValueStr() == "point_light")
		{
			TiXmlElement *attr = it->FirstChildElement();
			PointLight l;
			attr->Attribute("x" , &l.pos.x);
			attr->Attribute("y" , &l.pos.y);
			attr->Attribute("z" , &l.pos.z);
			attr = attr->NextSiblingElement();
			attr->Attribute("r" , &l.color.r);
			attr->Attribute("g" , &l.color.g);
			attr->Attribute("b" , &l.color.b);

			addLight(l);
		}
        else if (it->ValueStr() == "area_light")
        {
            TiXmlElement *attr = it->FirstChildElement();
            Mesh mesh;
            std::string filename = attr->Attribute("path");
            mesh.load(filename.c_str());

            Color3 color;
            attr = attr->NextSiblingElement();
            attr->Attribute("r" , &color.r);
            attr->Attribute("g" , &color.g);
            attr->Attribute("b" , &color.b);
            
            Triangle t;
			for (int i = 0; i < mesh.triangles.size(); i++)
			{
				t = Triangle(mesh.vertices[mesh.triangles[i].v[0]] ,
					mesh.vertices[mesh.triangles[i].v[1]] ,
					mesh.vertices[mesh.triangles[i].v[2]]);
                t.getMaterial().bxdf->kd = color;
				areaLightlist.push_back(t);
			}
        }
		else if (it->ValueStr() == "object")
		{
			TiXmlElement *attr = it->FirstChildElement();
			Mesh mesh;
			std::string filename = attr->Attribute("path");
			mesh.load(filename.c_str());

			attr = attr->NextSiblingElement();
			Color3 diffuse;
			attr->Attribute("r" , &diffuse.r);
			attr->Attribute("g" , &diffuse.g);
			attr->Attribute("b" , &diffuse.b);

            attr = attr->NextSiblingElement();
            Color3 specular;
            attr->Attribute("r" , &specular.r);
            attr->Attribute("g" , &specular.g);
            attr->Attribute("b" , &specular.b);

			attr = attr->NextSiblingElement();
			Real shininess = 0.0;
			attr->Attribute("shininess" , &shininess);

            attr = attr->NextSiblingElement();
            Real transparency = 0.0;
            attr->Attribute("transparency" , &transparency);

            attr = attr->NextSiblingElement();
            Real refractionIndex = 0.0;
            attr->Attribute("refractionIndex" , &refractionIndex);
            
			Triangle *t;
			for (int i = 0; i < mesh.triangles.size(); i++)
			{
				t = new Triangle(mesh.vertices[mesh.triangles[i].v[0]] ,
					mesh.vertices[mesh.triangles[i].v[1]] ,
					mesh.vertices[mesh.triangles[i].v[2]]);
				t->getMaterial().bxdf->kd = diffuse;
				t->getMaterial().bxdf->ks = specular;
				t->getMaterial().shininess = shininess;
                t->getMaterial().transparency = transparency;
                t->getMaterial().refractionIndex = refractionIndex;

				addGeometry(t);
			}
		}
		else if (it->ValueStr() == "sphere")
		{
            TiXmlElement *attr = it->FirstChildElement();
            Vector3 origin;
			attr->Attribute("x" , &origin.x);
            attr->Attribute("y" , &origin.y);
            attr->Attribute("z" , &origin.z);

            attr = attr->NextSiblingElement();
            Real radius;
            attr->Attribute("r" , &radius);

			attr = attr->NextSiblingElement();
			Color3 diffuse;
			attr->Attribute("r" , &diffuse.r);
			attr->Attribute("g" , &diffuse.g);
			attr->Attribute("b" , &diffuse.b);

            attr = attr->NextSiblingElement();
            Color3 specular;
            attr->Attribute("r" , &specular.r);
            attr->Attribute("g" , &specular.g);
            attr->Attribute("b" , &specular.b);

			attr = attr->NextSiblingElement();
			Real shininess = 0.0;
			attr->Attribute("shininess" , &shininess);

            attr = attr->NextSiblingElement();
            Real transparency = 0.0;
            attr->Attribute("transparency" , &transparency);

            attr = attr->NextSiblingElement();
            Real refractionIndex = 0.0;
            attr->Attribute("refractionIndex" , &refractionIndex);

            Sphere *s;
			s = new Sphere(origin , radius);
			s->getMaterial().bxdf->kd = diffuse;
			s->getMaterial().bxdf->ks = specular;
			s->getMaterial().shininess = shininess;
			s->getMaterial().transparency = transparency;
			s->getMaterial().refractionIndex = refractionIndex;

			addGeometry(s);
		}
		it = it->NextSiblingElement();
	}
}

void Scene::init(char* filename , Parameters& para)
{
	pointLightNum = para.POINT_LIGHT_NUM;
	
	//loadScene();
	loadScene(filename);

    /* sample point light from area light */
    lightlist = samplePointsOnAreaLight(areaLightlist , pointLightNum);
    
	kdtree.init(objlist);
	kdtree.buildTree(kdtree.root , 1);
}
