#include "testTransform.h"

static FILE *fp = fopen("testTransform.txt" , "w");

void testIdentityTransform()
{
	Transform t = identity();
	t.print(fp);
}

void testRotateTransform()
{
	Vector3 v = Vector3(1 , 0 , 0);
	Transform t = rotateX(90);
	printVector3(fp , t.tVector(v));
	t = rotateY(90);
	printVector3(fp , t.tVector(v));
	t = rotateZ(90);
	printVector3(fp , t.tVector(v));
	t = rotate(90 , Vector3(1 , 1 , 0));
	printVector3(fp , t.tVector(v));
}

void testAll()
{
	testIdentityTransform();
	testRotateTransform();
}