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
		Vector3(-0.0439815f, -4.12529f, 0.222539f),
		Vector3(0.00688625f, 0.998505f, -0.0542161f),
		Vector3(3.73896e-4f, 0.0542148f, 0.998529f),
		512 , 512 , 45);

	camera.worldToRaster.print(fp);
	camera.rasterToWorld.print(fp);
	Transform res = camera.worldToRaster * camera.rasterToWorld;
	res.print(fp);
}

void testAll()
{
	//testIdentityTransform();
	//testRotateTransform();
	testCamera();
}