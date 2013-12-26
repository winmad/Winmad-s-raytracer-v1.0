#include "scene.h"
#include "../sampler/sampler.h"
#include "../tinyxml/tinyxml.h"

void Scene::addGeometry(Geometry *g)
{
	objs.push_back(g);
	totArea += g->getArea();
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
	
    g = kdtreeAccel.traverse(ray , kdtreeAccel.root);
    if (g != NULL)
        g->hit(ray , inter);
	
// 	Real tmp = INF;
// 	for (int i = 0; i < objs.size(); i++)
// 	{
// 		objs[i]->hit(ray , inter);
// 		if (inter.t < tmp)
// 		{
// 			tmp = inter.t;
// 			g = objs[i];
// 		}
// 	}
// 	if (g != NULL)
// 		g->hit(ray , inter);
		
    return g;
}

bool Scene::intersect(const Ray& ray)
{
    Geometry *g = NULL;
    g = kdtreeAccel.traverse(ray , kdtreeAccel.root);
    if (g == NULL)
        return 0;
    else
        return 1;
}

Real Scene::shadowRayTest(const Ray& ray , const Vector3& p)
{
	Geometry *g = kdtreeAccel.traverse(ray , kdtreeAccel.root);
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
	
	Real largeRadius = 0.8f;
	Vector3 center = (cb[0] + cb[1] + cb[4] + cb[5]) * 0.25f +
		Vector3(0 , 0 , largeRadius);
	addGeometry(new Sphere(center , largeRadius , 6));
	
	// balls - left and right
	/*
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
	*/
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

static void readXYZ(TiXmlElement *attr , double& x , double& y , double& z)
{
	attr->Attribute("x" , &x);
	attr->Attribute("y" , &y);
	attr->Attribute("z" , &z);
}

static void readRGB(TiXmlElement *attr , double& r , double& g , double& b)
{
	attr->Attribute("r" , &r);
	attr->Attribute("g" , &g);
	attr->Attribute("b" , &b);
}

static void readVector3(TiXmlElement *attr , Vector3& v)
{
	double x , y , z;
	readXYZ(attr , x , y , z);
	v = Vector3(x , y , z);
}

static void readColor3(TiXmlElement *attr , Color3& c)
{
	double r , g , b;
	readRGB(attr , r , g , b);
	c = Color3(r , g , b);
}

// load .scene, xml format
void Scene::loadScene(char* filename)
{
	TiXmlDocument doc = TiXmlDocument(filename);
	doc.LoadFile();

	TiXmlElement *root = doc.RootElement();
	
	TiXmlElement *it = root->FirstChildElement();

	TiXmlElement *attr = NULL;

	std::vector<tinyobj::shape_t> shapes;

	double x , y , z , r , g , b;

	while (it)
	{
		if (it->ValueStr() == "camera")
		{
			// position
			attr = it->FirstChildElement();
			readVector3(attr , camera.pos);

			// forward
			attr = attr->NextSiblingElement();
			readVector3(attr , camera.forward);

			// up
			attr = attr->NextSiblingElement();
			readVector3(attr , camera.up);
			
			// resolution
			attr = attr->NextSiblingElement();
			attr->Attribute("height" , &x);
			attr->Attribute("width" , &y);
			camera.xResolution = x;
			camera.yResolution = y;

			// horizontal FOV
			attr = attr->NextSiblingElement();
			attr->Attribute("horizontalFOV" , &x);
			camera.horizontalFOV = x;

			camera.setup(camera.pos , camera.forward , camera.up ,
				camera.xResolution , camera.yResolution , camera.horizontalFOV);
		}
		else if (it->ValueStr() == "material")
		{
			Material mat;
			attr = it->FirstChildElement();

			// diffuse
			readColor3(attr , mat.diffuse);

			// glossy(phong)
			attr = attr->NextSiblingElement();
			readColor3(attr , mat.phong);

			// specular
			attr = attr->NextSiblingElement();
			readColor3(attr , mat.specular);

			// phong power
			attr = attr->NextSiblingElement();
			attr->Attribute("phongExp" , &x);
			mat.phongExp = x;

			// refraction index
			attr = attr->NextSiblingElement();
			attr->Attribute("refracIndex" , &x);
			mat.index = x;

			addMaterial(mat);
		}
		else if (it->ValueStr() == "object")
		{
			attr = it->FirstChildElement();
			std::string filename = attr->Attribute("path");
			tinyobj::LoadObj(shapes , filename.c_str());

			// material id
			int id;
			attr = attr->NextSiblingElement();
			attr->Attribute("matid" , &id);

			Triangle *t;
			
			Vector3 p[3];

			for (int i = 0; i < shapes.size(); i++)
			{
				for (int f = 0; f < shapes[i].mesh.indices.size() / 3; f++)
				{
					for (int j = 0; j < 3; j++)
					{
						int v = shapes[i].mesh.indices[3 * f + j];
						for (int k = 0; k < 3; k++)
						{
							p[j][k] = shapes[i].mesh.positions[3 * v + k];
						}
					}
					if (shapes[i].name == "water")
					{
						Vector3 n = (p[1] - p[0]) * (p[2] - p[0]);
						if (n.y < EPS)
						{
							Vector3 tmp = p[2];
							p[2] = p[0];
							p[0] = tmp;
						}
					}
					t = new Triangle(p[0] , p[1] , p[2] , id);
					addGeometry(t);
				}
			}
		}
		else if (it->ValueStr() == "sphere")
		{
            attr = it->FirstChildElement();
            
			// origin
			Vector3 origin;
			readVector3(attr , origin);

			// radius
			attr = attr->NextSiblingElement();
			double radius;
			attr->Attribute("radius" , &radius);

			// material id
			attr = attr->NextSiblingElement();
			int id;
			attr->Attribute("matid" , &id);

            Sphere *s;
			s = new Sphere(origin , radius , id);
			addGeometry(s);
		}
		else if (it->ValueStr() == "area_light")
		{
			attr = it->FirstChildElement();
			std::string filename = attr->Attribute("path");
			tinyobj::LoadObj(shapes , filename.c_str());

			// light intensity
			Color3 intensity;
			attr = attr->NextSiblingElement();
			readColor3(attr , intensity);

			AbstractLight *l;
			Triangle *t;

			Vector3 p[3];

			for (int i = 0; i < shapes.size(); i++)
			{
				for (int f = 0; f < shapes[i].mesh.indices.size() / 3; f++)
				{
					for (int j = 0; j < 3; j++)
					{
						int v = shapes[i].mesh.indices[3 * f + j];
						for (int k = 0; k < 3; k++)
						{
							p[j][k] = shapes[i].mesh.positions[3 * v + k];
						}
					}
					l = new AreaLight(p[0] , p[1] , p[2] , intensity);

					t = new Triangle(p[0] , p[1] , p[2] , -(f + 1));
					addLight(l);
					addGeometry(t);
				}
			}
		}
		else if (it->ValueStr() == "homo_media")
		{
			attr = it->FirstChildElement();

			HomogeneousVolumeDensity *homoMedia;
			homoMedia = new HomogeneousVolumeDensity();

			// sigma_a
			readColor3(attr , homoMedia->sigA);

			// sigma_s
			attr = attr->NextSiblingElement();
			readColor3(attr , homoMedia->sigS);

			// emit
			attr = attr->NextSiblingElement();
			readColor3(attr , homoMedia->le);

			// g
			attr = attr->NextSiblingElement();
			attr->Attribute("g" , &x);
			homoMedia->g = x;

			// bounding box
			attr = attr->NextSiblingElement();
			readVector3(attr , homoMedia->box.l);
			attr = attr->NextSiblingElement();
			readVector3(attr , homoMedia->box.r);

			volume = homoMedia;
		}

		it = it->NextSiblingElement();
	}
}

void Scene::init(char* filename , Parameters& para)
{
	//loadScene();
	loadScene(filename);

    /* sample point light from area light */
    //lightlist = samplePointsOnAreaLight(areaLightlist , pointLightNum);
    
	if (objs.size() > 0)
	{
		kdtreeAccel.init(objs);
		kdtreeAccel.buildTree(kdtreeAccel.root , 1);

		// build scene sphere
		Vector3 diag = kdtreeAccel.root->box.r - kdtreeAccel.root->box.l;
		Real diameter2 = diag.sqrLength();
		sceneSphere.sceneCenter = (kdtreeAccel.root->box.l + kdtreeAccel.root->box.r) * 0.5f;
		sceneSphere.sceneRadius = std::sqrt(diameter2) * 0.5f;
		sceneSphere.invSceneRadiusSqr = 1.f / diameter2;
	}
}
