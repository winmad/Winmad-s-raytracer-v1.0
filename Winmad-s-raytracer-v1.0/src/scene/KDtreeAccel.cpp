#include "KDtreeAccel.h"

static int cmp_sort_event(const void *p1 , const void *p2)
{
	Event e1 = *(Event*)p1 , e2 = *(Event*)p2;
	if (cmp(e1.pos - e2.pos) != 0)
		return cmp(e1.pos - e2.pos);
	else
		return (int)e1.type - (int)e2.type;
}

void KDtreeAccel::init(const std::vector<Geometry*>& _objlist)
{
	int _totObjNum = _objlist.size();
	int totObjNum = _totObjNum;
	depMax = (int)(1.2 * log((double)totObjNum) + 2.0);
	
	int MAX_TODO = depMax + 5;
	todo = new KDTodo[MAX_TODO];

	root = new KDtreeAccelNode();

	root->objNum = totObjNum;
	root->objlist = new Geometry*[root->objNum];
	for (int i = 0; i < totObjNum; i++)
		root->objlist[i] = _objlist[i];

	for (int i = 0; i < 3; i++)
	{
		root->eventNum[i] = 0;
		root->e[i] = new Event[2 * totObjNum];
		for (int j = 0; j < totObjNum; j++)
		{
			Real st = root->objlist[j]->box.l[i];
			Real ed = root->objlist[j]->box.r[i];
			root->e[i][root->eventNum[i]].type = Event::Start;
			root->e[i][root->eventNum[i]].pos = st;
			root->e[i][root->eventNum[i]].index = j;
			root->eventNum[i]++;
			root->e[i][root->eventNum[i]].type = Event::End;
			root->e[i][root->eventNum[i]].pos = ed;
			root->e[i][root->eventNum[i]].index = j;
			root->eventNum[i]++;
		}
		qsort(root->e[i] , root->eventNum[i] , sizeof(Event) , cmp_sort_event);
	}
	
	root->box.l.x = root->e[0][0].pos;
	root->box.l.y = root->e[1][0].pos;
	root->box.l.z = root->e[2][0].pos;
	root->box.r.x = root->e[0][root->eventNum[0] - 1].pos;
	root->box.r.y = root->e[1][root->eventNum[1] - 1].pos;
	root->box.r.z = root->e[2][root->eventNum[2] - 1].pos;

	root->left = root->right = NULL;
	root->axis = -1;
}

static Real SA(const Vector3& v)
{
	return 2 * (v.x * v.y + v.x * v.z + v.y * v.z);
}

static Real SAH(KDtreeAccelNode *tr , int axis , Real plane , 
				int NL , int NR)
{
	Vector3 vl , vr , v;
	AABB box = tr->box;
	vl = vr = v = box.r - box.l;
	if (axis == 0)
		vl.x = plane - box.l.x , vr.x = box.r.x - plane;
	if (axis == 1)
		vl.y = plane - box.l.y , vr.y = box.r.y - plane;
	if (axis == 2)
		vl.z = plane - box.l.z , vr.z = box.r.z - plane;
	Real lambda = 1.0f;
	if (NL == 0 || NR == 0)
		lambda = 0.8f;
	return (lambda / SA(v)) * (SA(vl) * NL + SA(vr) * NR);
}

static void findSplitPlane(KDtreeAccelNode *tr)
{
	Real cost = INF;
	tr->axis = -1;
	for (int axis = 0; axis < 3; axis++)
	{
		int nl , np , nr;
		nl = 0; np = 0; nr = tr->objNum;
		int i = 0;
		Real now;
		while (i < tr->eventNum[axis])
		{
			int p_end , p_start;
			p_end = p_start = 0;
			now = tr->e[axis][i].pos;
			while (i < tr->eventNum[axis] && tr->e[axis][i].pos == now)
			{
				if (tr->e[axis][i].type == Event::End)
					p_end++;
				if (tr->e[axis][i].type == Event::Start)
					p_start++;
				i++;
			}
			nr -= p_end;
			Real tmp = SAH(tr , axis , now , nl , nr);
			if (cmp(tmp - cost) < 0)
			{
				cost = tmp;
				tr->splitPlane = now;
				tr->axis = axis;
			}
			nl += p_start;
		}
	}
}

void KDtreeAccel::buildTree(KDtreeAccelNode *tr , int dep)
{
	if (dep > depMax)
		return;
	if (tr->objNum <= 1)
		return;

	findSplitPlane(tr);

	tr->div = new DivType[tr->objNum];
	tr->left = new KDtreeAccelNode;
	tr->right = new KDtreeAccelNode;
	KDtreeAccelNode *l , *r;
	l = tr->left;
	r = tr->right;

	int axis = tr->axis;

	int nl , nr , nb;
	nl = nr = nb = 0;
	for (int i = 0; i < tr->objNum; i++)
	{
		Real st = tr->objlist[i]->box.l[axis];
		Real ed = tr->objlist[i]->box.r[axis];
		if (cmp(ed - tr->splitPlane) <= 0)
		{
			tr->div[i] = LeftOnly;
			nl++;
		}
		else if (cmp(tr->splitPlane - st) <= 0)
		{
			tr->div[i] = RightOnly;
			nr++;
		}
		else
		{
			tr->div[i] = Both;
			nb++;
		}
	}

	l->objNum = nl + nb;
	l->objlist = new Geometry*[l->objNum];
	l->left = l->right = NULL;
	l->axis = -1;

	r->objNum = nb + nr;
	r->objlist = new Geometry*[r->objNum];
	r->left = r->right = NULL;
	r->axis = -1;

	int pl , pr;
	pl = pr = 0;
	int *index_to_l , *index_to_r;
	index_to_l = new int[tr->objNum];
	index_to_r = new int[tr->objNum];
	for (int i = 0; i < tr->objNum; i++)
	{
		if (tr->div[i] == LeftOnly)
		{
			index_to_l[i] = pl;
			l->objlist[pl++] = tr->objlist[i];
		}
		else if (tr->div[i] == RightOnly)
		{
			index_to_r[i] = pr;
			r->objlist[pr++] = tr->objlist[i];
		}
		else
		{
			index_to_l[i] = pl;
			l->objlist[pl++] = tr->objlist[i];
			index_to_r[i] = pr;
			r->objlist[pr++] = tr->objlist[i];
		}
	}

	Event e;
	for (int i = 0; i < 3; i++)
	{
		l->eventNum[i] = r->eventNum[i] = 0;
		l->e[i] = new Event[l->objNum * 2];
		r->e[i] = new Event[r->objNum * 2];
		if (i != axis)
		{
			for (int j = 0; j < tr->eventNum[i]; j++)
			{
				if (tr->div[tr->e[i][j].index] == LeftOnly)
				{
					e.pos = tr->e[i][j].pos;
					e.type = tr->e[i][j].type;
					e.index = index_to_l[tr->e[i][j].index];
					l->e[i][l->eventNum[i]++] = e;
				}
				else if (tr->div[tr->e[i][j].index] == RightOnly)
				{
					e.pos = tr->e[i][j].pos;
					e.type = tr->e[i][j].type;
					e.index = index_to_r[tr->e[i][j].index];
					r->e[i][r->eventNum[i]++] = e;
				}
				else if (tr->div[tr->e[i][j].index] == Both)
				{
					e.pos = tr->e[i][j].pos;
					e.type = tr->e[i][j].type;
					e.index = index_to_l[tr->e[i][j].index];
					l->e[i][l->eventNum[i]++] = e;

					e.pos = tr->e[i][j].pos;
					e.type = tr->e[i][j].type;
					e.index = index_to_r[tr->e[i][j].index];
					r->e[i][r->eventNum[i]++] = e;
				}
			}
		}
		else
		{
			for (int j = 0; j < tr->eventNum[i]; j++)
			{
				if (tr->div[tr->e[i][j].index] == LeftOnly)
				{
					e.pos = tr->e[i][j].pos;
					e.type = tr->e[i][j].type;
					e.index = index_to_l[tr->e[i][j].index];
					l->e[i][l->eventNum[i]++] = e;
				}
				else if (tr->div[tr->e[i][j].index] == RightOnly)
				{
					e.pos = tr->e[i][j].pos;
					e.type = tr->e[i][j].type;
					e.index = index_to_r[tr->e[i][j].index];
					r->e[i][r->eventNum[i]++] = e;
				}
				else if (tr->div[tr->e[i][j].index] == Both)
				{
					if (tr->e[i][j].type == Event::End)
					{
						e.pos = tr->splitPlane;
						e.type = tr->e[i][j].type;
						e.index = index_to_l[tr->e[i][j].index];
						l->e[i][l->eventNum[i]++] = e;

						e.pos = tr->e[i][j].pos;
						e.type = tr->e[i][j].type;
						e.index = index_to_r[tr->e[i][j].index];
						r->e[i][r->eventNum[i]++] = e;
					}
					else if (tr->e[i][j].type == Event::Start)
					{
						e.pos = tr->e[i][j].pos;
						e.type = tr->e[i][j].type;
						e.index = index_to_l[tr->e[i][j].index];
						l->e[i][l->eventNum[i]++] = e;

						e.pos = tr->splitPlane;
						e.type = tr->e[i][j].type;
						e.index = index_to_r[tr->e[i][j].index];
						r->e[i][r->eventNum[i]++] = e;
					}
				}
			}
		}
	}
	if (l->objNum > 0)
	{
		l->box.l.x = l->e[0][0].pos;
		l->box.l.y = l->e[1][0].pos;
		l->box.l.z = l->e[2][0].pos;
		l->box.r.x = l->e[0][l->eventNum[0] - 1].pos;
		l->box.r.y = l->e[1][l->eventNum[1] - 1].pos;
		l->box.r.z = l->e[2][l->eventNum[2] - 1].pos;
	}
	if (r->objNum > 0)
	{
		r->box.l.x = r->e[0][0].pos;
		r->box.l.y = r->e[1][0].pos;
		r->box.l.z = r->e[2][0].pos;
		r->box.r.x = r->e[0][r->eventNum[0] - 1].pos;
		r->box.r.y = r->e[1][r->eventNum[1] - 1].pos;
		r->box.r.z = r->e[2][r->eventNum[2] - 1].pos;
	}
	/*
	for (int i = 0; i < 3; i++) delete[] tr->e[i];
	delete[] tr->div;
	delete[] index_to_l;
	delete[] index_to_r;
	*/
	buildTree(l , dep + 1);
	buildTree(r , dep + 1);
}

Geometry* KDtreeAccel::traverse(const Ray& ray , KDtreeAccelNode *tr)
{
	Real tmin , tmax;
	if (!tr->box.hit(ray , tmin , tmax))
		return NULL;

	Vector3 invDir(1.f / ray.dir.x , 1.f / ray.dir.y , 1.f / ray.dir.z);

	int todoPos = 0;
	Geometry* res = NULL;
	Real tmp = INF;

	while (tr != NULL)
	{
		if (ray.tmax < tmin)
			break;
		if (tr->axis != -1)
		{
			int axis = tr->axis;
			Real t = (tr->splitPlane - ray.origin[axis]) * invDir[axis];

			KDtreeAccelNode *near , *far;
			int belowFirst = (ray.origin[axis] < tr->splitPlane) ||
				(ray.origin[axis] == tr->splitPlane && ray.dir[axis] <= 0);

			if (belowFirst)
			{
				near = tr->left;
				far = tr->right;
			}
			else
			{
				near = tr->right;
				far = tr->left;
			}

			if (t > tmax || t <= 0)
				tr = near;
			else if (t < tmin)
				tr = far;
			else
			{
				todo[todoPos].node = far;
				todo[todoPos].tmin = t;
				todo[todoPos].tmax = tmax;
				++todoPos;
				tr = near;
				tmax = t;
			}
		}
		else 
		{
			Intersection inter;
			inter.t = INF;
			for (int i = 0; i < tr->objNum; i++)
			{
				if (tr->objlist[i]->hit(ray , inter))
				{
					if (cmp(inter.t - tmp) < 0)
					{
						tmp = inter.t;
						res = tr->objlist[i];
					}
				}
			}

			if (todoPos > 0)
			{
				--todoPos;
				tr = todo[todoPos].node;
				tmin = todo[todoPos].tmin;
				tmax = todo[todoPos].tmax;
			}
			else
				break;
		}
	}

	return res;
}

void print_tree(FILE *fp , KDtreeAccelNode *tr)
{
	if (tr == NULL)
		return;
	fprintf(fp , "----------------------------\n");
	fprintf(fp , "objNum = %d\n" , tr->objNum);
	fprintf(fp , "objects:\n");
	for (int i = 0; i < tr->objNum; i++)
	{
		fprintf(fp , "obj #%d's box = (%.3lf,%.3lf,%.3lf),(%.3lf,%.3lf,%.3lf)\n" ,
			i , tr->objlist[i]->box.l.x , tr->objlist[i]->box.l.y , 
			tr->objlist[i]->box.l.z , tr->objlist[i]->box.r.x ,
			tr->objlist[i]->box.r.y , tr->objlist[i]->box.r.z);
	}
	fprintf(fp , "\nNode's box = (%.3lf,%.3lf,%.3lf),(%.3lf,%.3lf,%.3lf)\n" ,
		tr->box.l.x , tr->box.l.y , tr->box.l.z , tr->box.r.x , tr->box.r.y , tr->box.r.z);

	fprintf(fp , "\nSplit plane: axis = %d , pos = %.3lf\n" ,
		(int)tr->axis , tr->splitPlane);

	/*
	fprintf(fp , "\nX-axis events:\n");
	for (int i = 0; i < tr->eventNum[0]; i++)
	{
		fprintf(fp , "event #%d: pos = %.3lf , type = %d , obj index = %d\n" ,
			i , tr->e[0][i].pos , (int)tr->e[0][i].type , tr->e[0][i].index);
	}

	fprintf(fp , "\nY-axis events:\n");
	for (int i = 0; i < tr->eventNum[0]; i++)
	{
		fprintf(fp , "event #%d: pos = %.3lf , type = %d , obj index = %d\n" ,
			i , tr->e[1][i].pos , (int)tr->e[1][i].type , tr->e[1][i].index);
	}

	fprintf(fp , "\nZ-axis events:\n");
	for (int i = 0; i < tr->eventNum[0]; i++)
	{
		fprintf(fp , "event #%d: pos = %.3lf , type = %d , obj index = %d\n" ,
			i , tr->e[2][i].pos , (int)tr->e[2][i].type , tr->e[2][i].index);
	}
	*/
	
	print_tree(fp , tr->left);
	print_tree(fp , tr->right);
}
