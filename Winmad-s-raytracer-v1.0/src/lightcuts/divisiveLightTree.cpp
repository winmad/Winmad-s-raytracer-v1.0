#include "divisiveLightTree.h"
#include "../math/drand48.h"

static int cmp_sort_vl(const void *a , const void *b)
{
	LightPosSorted p1 = *(LightPosSorted*)a;
	LightPosSorted p2 = *(LightPosSorted*)b;
	return cmp(p1.pos - p2.pos);
}

void LightTree::init(const std::vector<VirtualLight>& lights)
{
	maxCutSize = MAX_CUT_SIZE;

	root = new LightTreeNode();
	root->lights.clear();
	for (int i = 0; i < lights.size(); i++)
		root->lights.push_back(lights[i]);

	int N = lights.size();
	for (int i = 0; i < 3; i++)
	{
		root->p[i] = new LightPosSorted[N];

		for (int j = 0; j < N; j++)
		{
			root->p[i][j].pos = (lights[j].p)[i];
			root->p[i][j].index = j;
		}

		qsort(root->p[i] , N , sizeof(LightPosSorted) , cmp_sort_vl);
	}
	
	root->axis = -1;

	root->left = root->right = NULL;
}

void LightTree::buildTree(LightTreeNode *tr)
{
	if (tr->lights.size() == 1)
	{
		tr->axis = -1;
		tr->vl = VirtualLight(tr->lights[0].p , tr->lights[0].alpha);
		tr->left = tr->right = NULL;
		tr->lights.clear();
		return;
	}
	int N = tr->lights.size();

	tr->box = AABB(tr->p[0][0].pos , tr->p[0][N - 1].pos ,
		tr->p[1][0].pos , tr->p[1][N - 1].pos ,
		tr->p[2][0].pos , tr->p[2][N - 1].pos);

	// choose representative virtual light
	tr->totIntensity = 0;
	Color3 alpha = Color3(0.0 , 0.0 , 0.0);
	for (int i = 0; i < N; i++)
	{
		tr->totIntensity += tr->lights[i].intensity;
		alpha = alpha + tr->lights[i].alpha * tr->lights[i].intensity;
	}
	alpha = alpha / tr->totIntensity;

	Real rn = drand48();
	Real tmp = 0;
	for (int i = 0; i < N; i++)
	{
		tmp += tr->lights[i].intensity / tr->totIntensity;
		if (cmp(rn - tmp) <= 0)
		{
			tr->vl = VirtualLight(tr->lights[i].p , alpha);
			break;
		}
	}

	// choose longest axis to divide
	int axis = 0;
	for (int i = 1; i < 3; i++)
	{
		if (tr->box.r[i] - tr->box.l[i] >
			tr->box.r[axis] - tr->box.l[axis])
			axis = i;
	}
	tr->axis = axis;

	// choose median
	int mid = N / 2;

	tr->left = new LightTreeNode();
	tr->right = new LightTreeNode();
	
	tr->div = new int[N];

	for (int i = 0; i < mid; i++)
		tr->div[tr->p[axis][i].index] = 0;
	for (int i = mid; i < N; i++)
		tr->div[tr->p[axis][i].index] = 1;

	LightTreeNode *left = tr->left , *right = tr->right;

	left->axis = right->axis = -1;

	int *index_to_l , *index_to_r;
	index_to_l = new int[N];
	index_to_r = new int[N];

	for (int i = 0; i < mid; i++)
	{
		left->lights.push_back(tr->lights[tr->p[axis][i].index]);
		index_to_l[tr->p[axis][i].index] = i;
	}
	for (int i = mid; i < N; i++)
	{
		right->lights.push_back(tr->lights[tr->p[axis][i].index]);
		index_to_r[tr->p[axis][i].index] = i - mid;
	}

	for (axis = 0; axis < 3; axis++)
	{
		int lp = 0 , rp = 0;
		left->p[axis] = new LightPosSorted[mid];
		right->p[axis] = new LightPosSorted[N - mid];
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

	tr->lights.clear();
	for (int i = 0; i < 3; i++)
		delete[] tr->p[i];
	delete[] tr->div;
	delete[] index_to_l;
	delete[] index_to_r;

	buildTree(left);
	buildTree(right);
}

static Color3 calcRadiance(const Vector3& p , const Vector3& n ,
	const Vector3& wo , Geometry *g , const Vector3& lightP , 
	const Color3& alpha)
{
	Vector3 wi = lightP - p;
	wi.normalize();
	Color3 brdf = g->getMaterial().bxdf->calcBrdf(wi , wo , n);
	return ((alpha * clampVal(wi ^ n , 0.0 , 1.0)) | brdf);
}

static Real calcVisibleTerm(const Vector3& p , const VirtualLight& vl ,
	Scene& scene)
{
	Vector3 wi;
	Ray ray;
	Real vis;

	wi = vl.p - p;
	wi.normalize();

	ray = Ray(vl.p - wi * (eps * 10.0) , -wi);
	if (cmp(scene.shadowRayTest(ray , p)) == 0)
		vis = 0.0;
	else 
		vis = 1.0;

	return vis;
}

static Color3 calcMaxErrorBound(const Vector3& p , const Vector3& n ,
	const Vector3& wo , Geometry *g , LightTreeNode *tr)
{
	Color3 alpha = tr->vl.alpha;
	Color3 res = Color3(0.0 , 0.0 , 0.0);
	Real x[2] , y[2] , z[2];
	x[0] = tr->box.l[0]; x[1] = tr->box.r[0];
	y[0] = tr->box.l[1]; y[1] = tr->box.r[1];
	z[0] = tr->box.l[2]; z[1] = tr->box.r[2];
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < 2; k++)
			{
				Vector3 lightP = Vector3(x[i] , y[j] , z[k]);
				Color3 tmp = calcRadiance(p , n , wo , g , lightP , alpha);
				res.r = std::max(res.r , tmp.r);
				res.g = std::max(res.g , tmp.g);
				res.b = std::max(res.b , tmp.b);
			}
		}
	}
	Color3 tmp = calcRadiance(p , n , wo , g , tr->vl.p , alpha);
	res.r = std::max(res.r , tmp.r);
	res.g = std::max(res.g , tmp.g);
	res.b = std::max(res.b , tmp.b);
	return res;
}

Color3 LightTree::findLightCuts(const Vector3& p , const Vector3& n , 
	const Vector3& wo , Geometry *g , Scene& scene ,
	std::priority_queue<LightCluster>& cut)
{
	LightCluster c = LightCluster(root);

	Color3 totRadiance = Color3(0.0 , 0.0 , 0.0);

	Real vis = calcVisibleTerm(p , root->vl , scene);

	totRadiance = totRadiance + calcRadiance(p , n , wo , g , 
		root->vl.p , root->vl.alpha) * vis;
	c.contrib = totRadiance;
	Color3 err = calcMaxErrorBound(p , n , wo , g , c.tr);
	c.maxErrorRadiance = err.intensity();
	cut.push(c);

	// save for temporary
	std::vector<LightCluster> leaves;

	while (!cut.empty())
	{
		c = cut.top();
		if (cmp(c.maxErrorRadiance 
			- 0.02 * totRadiance.intensity()) <= 0 ||
			cut.size() >= maxCutSize)
			break;
		// refine
		cut.pop();
		if (c.tr->axis == -1)
		{
			leaves.push_back(c);
			continue;
		}
		totRadiance = totRadiance - c.contrib;

		LightCluster c1 , c2;
		c1 = LightCluster(c.tr->left);
		c1.contrib = calcRadiance(p , n , wo , g , 
			c.tr->left->vl.p , c.tr->left->vl.alpha) *
			calcVisibleTerm(p , c.tr->left->vl , scene);
		err = calcMaxErrorBound(p , n , wo , g , c.tr->left);
		c1.maxErrorRadiance = err.intensity();

		c2 = LightCluster(c.tr->right);
		c2.contrib = calcRadiance(p , n , wo , g , 
			c.tr->right->vl.p , c.tr->right->vl.alpha) *
			calcVisibleTerm(p , c.tr->right->vl , scene);
		err = calcMaxErrorBound(p , n , wo , g , c.tr->right);
		c2.maxErrorRadiance = err.intensity();

		totRadiance = totRadiance + c1.contrib + c2.contrib;
		if (c.tr->left != NULL)
			cut.push(c1);
		if (c.tr->right != NULL)
			cut.push(c2);
	}
	for (int i = 0; i < leaves.size(); i++)
	{
		cut.push(leaves[i]);
		totRadiance = totRadiance + calcRadiance(p , n , wo , g , 
			leaves[i].vl.p , leaves[i].vl.alpha) * 
			calcVisibleTerm(p , leaves[i].vl , scene);
	}
	return totRadiance;
}