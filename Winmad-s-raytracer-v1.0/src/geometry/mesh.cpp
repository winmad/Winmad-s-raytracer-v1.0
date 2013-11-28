#include "mesh.h"

void Mesh::load(const char* filename)
{
	FILE* file = fopen(filename , "r");
	char ch;
	while (fscanf(file , "%c" , &ch) != EOF)
	{
		if (ch == '#')
		{
			while (ch != '\n') 
			{
				if (fscanf(file , "%c" , &ch) == EOF)
					break;
			}
			continue;
		}
		if (ch == '\n') continue;
		if (ch == 'v')
		{
			Vector3 v;
			fscanf(file , "%f %f %f\n" , &v.x , &v.y , &v.z);
			vertices.push_back(v);
			continue;
		}
		if (ch == 'f')
		{
			int _v[3];
			fscanf(file , "%d %d %d\n" , &_v[0] , &_v[1] , &_v[2]);
			_v[0]--; _v[1]--; _v[2]--;
			MeshTriangle t = MeshTriangle(_v[0] , _v[1] , _v[2]);
			triangles.push_back(t);
			continue;
		}
	}
	fclose(file);
}