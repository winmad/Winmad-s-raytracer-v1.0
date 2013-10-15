#include "vertexKDtree.h"

static Real get_projection(const Vector3& v , int axis)
{
	if (axis == 0)
		return v.x;
	if (axis == 1)
		return v.y;
	if (axis == 2)
		return v.z;
	return INF;
}

static int cmp_sort_point(const void *a , const void *b)
{
	VertexSorted p1 = *(VertexSorted*)a;
	VertexSorted p2 = *(VertexSorted*)b;
	return cmp(p1.pos - p2.pos);
}

void VertexKDtree::init(const std::vector<PathVertex>& vertices)
{
	totNum = vertices.size();
	depMax = (int)(1.2 * log((double)totNum) + 2.0);

	root = new VertexKDtreeNode();
	root->vertices.clear();
	for (int i = 0; i < vertices.size(); i++)
		root->vertices.push_back(vertices[i]);

	for (int i = 0; i < 3; i++)
	{
		root->p[i] = new VertexSorted[totNum];

		for (int j = 0; j < totNum; j++)
		{
			root->p[i][j].pos = get_projection(vertices[j].hitPos , i);
			root->p[i][j].index = j;
		}

		qsort(root->p[i] , totNum , sizeof(VertexSorted) , cmp_sort_point);
	}

	root->axis = -1;
	root->left = root->right = NULL;
}

void VertexKDtree::buildTree(VertexKDtreeNode *tr , int dep)
{
	if (dep > depMax)
		return;
	if (tr->vertices.size() <= 1)
		return;

	/* find median */
	int axis = dep % 3;
	int mid = tr->vertices.size() / 2;

	tr->axis = axis;

	tr->left = new VertexKDtreeNode();
	tr->right = new VertexKDtreeNode();

	tr->div = new int[tr->vertices.size()];

	for (int i = 0; i < mid; i++)
		tr->div[tr->p[axis][i].index] = 0;
	for (int i = mid + 1; i < tr->vertices.size(); i++)
		tr->div[tr->p[axis][i].index] = 1;
	tr->div[tr->p[axis][mid].index] = -1;

	VertexKDtreeNode *left = tr->left , *right = tr->right;
	int N = tr->vertices.size();

	left->axis = right->axis = -1;
	left->left = left->right = right->left = right->right = NULL;

	int *index_to_l , *index_to_r;

	index_to_l = new int[N];
	index_to_r = new int[N];

	for (int i = 0; i < mid; i++)
	{
		left->vertices.push_back(tr->vertices[tr->p[axis][i].index]);
		index_to_l[tr->p[axis][i].index] = i;
	}
	for (int i = mid + 1; i < N; i++)
	{
		right->vertices.push_back(tr->vertices[tr->p[axis][i].index]);
		index_to_r[tr->p[axis][i].index] = i - mid - 1;
	}

	for (axis = 0; axis < 3; axis++)
	{
		int lp = 0 , rp = 0;
		left->p[axis] = new VertexSorted[mid + 1];
		right->p[axis] = new VertexSorted[N - mid + 1];
		for (int i = 0; i < N; i++)
		{
			if (tr->div[tr->p[axis][i].index] == 0)
			{
				left->p[axis][lp].pos = tr->p[axis][i].pos;
				left->p[axis][lp].index = index_to_l[tr->p[axis][i].index];
				lp++;
			}
			else if (tr->div[tr->p[axis][i].index] == 1)
			{
				right->p[axis][rp].pos = tr->p[axis][i].pos;
				right->p[axis][rp].index = index_to_r[tr->p[axis][i].index];
				rp++;
			}
		}
	}

	PathVertex vertex = tr->vertices[tr->p[tr->axis][mid].index];
	tr->vertices.clear();
	tr->vertices.push_back(vertex);
	for (int i = 0; i < 3; i++)
		delete[] tr->p[i];
	delete[] tr->div;
	delete[] index_to_l;
	delete[] index_to_r;

	buildTree(left , dep + 1);
	buildTree(right , dep + 1);
}

void VertexKDtree::searchVertexInRadius(std::vector<PathVertex>& vertices , 
	VertexKDtreeNode *tr , const Vector3& pos , Real radius)
{
	if (tr == NULL)
		return;
	if (tr->vertices.size() <= 0)
		return;

	VertexKDtreeNode *near , *far;
	Real p_pos = get_projection(pos , tr->axis);
	Real split_pos = get_projection(tr->vertices[0].hitPos , tr->axis);
	Real delta = p_pos - split_pos;

	if (cmp(delta) <= 0)
	{
		near = tr->left;
		far = tr->right;
	}
	else
	{
		near = tr->right;
		far = tr->left;
	}

	searchVertexInRadius(vertices , near , pos , radius);

	Vector3 d = pos - tr->vertices[0].hitPos;
	Real dis = d.length();

	if (cmp(dis - radius) < 0)
	{
		vertices.push_back(tr->vertices[0]);
	}
	if (cmp(delta - radius) < 0)
		searchVertexInRadius(vertices , far , pos , radius);
}
