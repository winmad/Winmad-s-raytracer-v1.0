#include "test.h"

static FILE *fp = fopen("test.txt" , "w");

void testIdentityTransform()
{
	Transform t = identity();
	t.print(fp);
}

void testRotateTransform()
{
	Vector3 v = Vector3(1 , 0 , 0);
	Transform t = rotateX(90);
	Transform tInv = Transform(t.mInv);
	printVector3(fp , t.tVector(v));
	printVector3(fp , tInv.tVector(t.tVector(v)));
	t = rotateY(90);
	tInv = Transform(t.mInv);
	printVector3(fp , t.tVector(v));
	printVector3(fp , tInv.tVector(t.tVector(v)));
	t = rotateZ(90);
	tInv = Transform(t.mInv);
	printVector3(fp , t.tVector(v));
	printVector3(fp , tInv.tVector(t.tVector(v)));
	t = rotate(90 , Vector3(1 , 1 , 0));
	tInv = Transform(t.mInv);
	printVector3(fp , t.tVector(v));
	printVector3(fp , tInv.tVector(t.tVector(v)));
}

void testCamera()
{
	Camera camera;
	camera.setup(
		Vector3(0, 0, 1),
		Vector3(0, 0, -1),
		Vector3(0, 1, 0),
		512 , 512 , 90);

	Vector3 p(-1 , 0 , 0);
	Vector3 t1 = camera.worldToCamera.tPoint(p);
	Vector3 t2 = camera.worldToRaster.tPoint(p);
	Vector3 q(256 , 256 , 0);
	Vector3 t3 = camera.rasterToWorld.tPoint(q);
	printVector3(fp , t1);
	printVector3(fp , t2);
	printVector3(fp , t3);
}

void testAll()
{
	//testIdentityTransform();
	//testRotateTransform();
	testCamera();
}