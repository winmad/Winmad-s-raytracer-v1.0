#include "scene.h"
#include "../sampler/sampler.h"
#include "../tinyxml/tinyxml.h"

void Scene::addGeometry(Geometry *g)
{
	objs.push_back(g);
}

void Scene::addLight(AbstractLight *l)
{
	lights.push_back(l);
}

void Scene::addMaterial(Material mat)
{
	materials.push_back(mat);
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

bool Scene::occluded(const Vector3& p1 , const Vector3& dir , 
	const Vector3& p2)
{
	Ray ray(p1 , dir);
	Real res = shadowRayTest(ray , p2);

	if (cmp(res) == 0)
		return 1;
	else 
		return 0;
}

// build-in scene: conerll box
void Scene::loadScene()
{
	// camera
	camera.setup(Vector3(-0.0439815f, -4.12529f, 0.222539f),
		Vector3(0.00688625f, 0.998505f, -0.0542161f),
		Vector3(3.73896e-4f, 0.0542148f, 0.998529f),
		512 , 512 , 45);

	///////////////////////////////////
	// materials
	Material mat;
	materials.clear();

	// 0) light 1
	addMaterial(mat);
	// 1) light 2
	addMaterial(mat);

	// 2) glossy white floor
	mat.init();
	mat.diffuse = Color3(0.1f);
	mat.phong = Color3(0.7f);
	mat.phongExp = 90.f;
	addMaterial(mat);

	// 3) diffuse green left wall
	mat.init();
	mat.diffuse = Color3(0.156863f, 0.803922f, 0.172549f);
	addMaterial(mat);

	// 4) diffuse red right wall
	mat.init();
	mat.diffuse = Color3(0.803922f, 0.152941f, 0.152941f);
	addMaterial(mat);

	// 5) diffuse white back wall
	mat.init();
	mat.diffuse = Color3(0.803922f, 0.803922f, 0.803922f);
	addMaterial(mat);

	// 6) mirror ball
	mat.init();
	mat.specular = Color3(1.f);
	addMaterial(mat);

	// 7) glass ball
	mat.init();
	mat.specular  = Color3(1.f);
	mat.index = 1.6f;
	addMaterial(mat);

	// 8) diffuse blue wall (back wall for glossy floor)
	mat.init();
	mat.diffuse = Color3(0.156863f, 0.172549f, 0.803922f);
	addMaterial(mat);

	///////////////////////////////////
	// geometry
	objs.clear();

	// Cornell box
	Vector3 cb[8] = {
		Vector3(-1.27029f,  1.30455f, -1.28002f),
		Vector3( 1.28975f,  1.30455f, -1.28002f),
		Vector3( 1.28975f,  1.30455f,  1.28002f),
		Vector3(-1.27029f,  1.30455f,  1.28002f),
		Vector3(-1.27029f, -1.25549f, -1.28002f),
		Vector3( 1.28975f, -1.25549f, -1.28002f),
		Vector3( 1.28975f, -1.25549f,  1.28002f),
		Vector3(-1.27029f, -1.25549f,  1.28002f)
	};
	// floor
	addGeometry(new Triangle(cb[0] , cb[4] , cb[5] , 2));
	addGeometry(new Triangle(cb[5] , cb[1] , cb[0] , 2));
	// back wall
	addGeometry(new Triangle(cb[0] , cb[1] , cb[2] , 8));
	addGeometry(new Triangle(cb[2] , cb[3] , cb[0] , 8));
	// ceiling
	addGeometry(new Triangle(cb[2] , cb[6] , cb[7] , 5));
	addGeometry(new Triangle(cb[7] , cb[3] , cb[2] , 5));
	// left wall
	addGeometry(new Triangle(cb[3] , cb[7] , cb[4] , 3));
	addGeometry(new Triangle(cb[4] , cb[0] , cb[3] , 3));
	// right wall
	addGeometry(new Triangle(cb[1] , cb[5] , cb[6] , 4));
	addGeometry(new Triangle(cb[6] , cb[2] , cb[1] , 4));
	// balls - central
	/*
	Real largeRadius = 0.8f;
	Vector3 center = (cb[0] + cb[1] + cb[4] + cb[5]) * 0.25f +
		Vector3(0 , 0 , largeRadius);
	addGeometry(new Sphere(center , largeRadius , 6));
	*/
	// balls - left and right
	Real smallRadius = 0.5f;
	Vector3 leftWallCenter = (cb[0] + cb[4]) * 0.5f +
		Vector3(0 , 0 , smallRadius);
	Vector3 rightWallCenter = (cb[1] + cb[5]) * 0.5f +
		Vector3(0 , 0 , smallRadius);
	Real xlen = rightWallCenter.x - leftWallCenter.x;
	Vector3 leftBallCenter = leftWallCenter + 
		Vector3(2.f * xlen / 7.f , 0 , 0);
	Vector3 rightBallCenter = rightWallCenter -
		Vector3(2.f * xlen / 7.f , 0 , 0);
	addGeometry(new Sphere(leftBallCenter , smallRadius , 6));
	addGeometry(new Sphere(rightBallCenter , smallRadius , 7));
	// light box at ceiling
	Vector3 lb[8] = {
		Vector3(-0.25f,  0.25f, 1.26002f),
		Vector3( 0.25f,  0.25f, 1.26002f),
		Vector3( 0.25f,  0.25f, 1.28002f),
		Vector3(-0.25f,  0.25f, 1.28002f),
		Vector3(-0.25f, -0.25f, 1.26002f),
		Vector3( 0.25f, -0.25f, 1.26002f),
		Vector3( 0.25f, -0.25f, 1.28002f),
		Vector3(-0.25f, -0.25f, 1.28002f)
	};
	// Back wall
	addGeometry(new Triangle(lb[0], lb[2], lb[1], 5));
	addGeometry(new Triangle(lb[2], lb[0], lb[3], 5));
	// Left wall
	addGeometry(new Triangle(lb[3], lb[4], lb[7], 5));
	addGeometry(new Triangle(lb[4], lb[3], lb[0], 5));
	// Right wall
	addGeometry(new Triangle(lb[1], lb[6], lb[5], 5));
	addGeometry(new Triangle(lb[6], lb[1], lb[2], 5));
	// Front wall
	addGeometry(new Triangle(lb[4], lb[5], lb[6], 5));
	addGeometry(new Triangle(lb[6], lb[7], lb[4], 5));
	// Floor
	addGeometry(new Triangle(lb[0], lb[5], lb[4], -1));
	addGeometry(new Triangle(lb[5], lb[0], lb[1], -2));

	///////////////////////////////////
	// lights
	lights.clear();

	addLight(new AreaLight(lb[0] , lb[5] , lb[4] , 
		Color3(25.03329895614464f)));

	addLight(new AreaLight(lb[5] , lb[0] , lb[1] ,
		Color3(25.03329895614464f)));
}

// load .scene, xml format
void Scene::loadScene(char* filename)
{
	/*
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
	*/
}

void Scene::init(char* filename , Parameters& para)
{
	loadScene();
	//loadScene(filename);

    /* sample point light from area light */
    //lightlist = samplePointsOnAreaLight(areaLightlist , pointLightNum);
    
	kdtree.init(objs);
	kdtree.buildTree(kdtree.root , 1);

	// build scene sphere
	Vector3 diag = kdtree.root->box.r - kdtree.root->box.l;
	Real diameter2 = diag.sqrLength();
	sceneSphere.sceneCenter = (kdtree.root->box.l + kdtree.root->box.r) * 0.5f;
	sceneSphere.sceneRadius = std::sqrt(diameter2) * 0.5f;
	sceneSphere.invSceneRadiusSqr = 1.f / diameter2;
}
