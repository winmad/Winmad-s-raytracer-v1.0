#include "KDtree.h"

static Real get_projection(const Vector3& v , int axis)
{
	if (axis == 0)
		return v.x;
	if (axis == 1)
		return v.y;
	if (axis == 2)
		return v.z;
	return inf;
}

static KDtreeNode::Axes get_axis(const int &axis)
{
	if (axis == 0)
		return KDtreeNode::X_axis;
	else if (axis == 1)
		return KDtreeNode::Y_axis;
	else if (axis == 2)
		return KDtreeNode::Z_axis;
	else 
		return KDtreeNode::No_axis;
}

static int cmp_sort_event(const void *p1 , const void *p2)
{
	Event e1 = *(Event*)p1 , e2 = *(Event*)p2;
	if (cmp(e1.pos - e2.pos) != 0)
		return cmp(e1.pos - e2.pos);
	else
		return (int)e1.type - (int)e2.type;
}

void KDtree::init(const std::vector<Geometry*>& _objlist)
{
	int _totObjNum = _objlist.size();
	int totObjNum = _totObjNum;
	depMax = (int)(1.2 * log((double)totObjNum) + 2.0);
	root = new KDtreeNode();

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
			Real st = get_projection(root->objlist[j]->box.l , i);
			Real ed = get_projection(root->objlist[j]->box.r , i);
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
	root->axis = KDtreeNode::No_axis;
}

static Real SA(const Vector3& v)
{
	return 2 * (v.x * v.y + v.x * v.z + v.y * v.z);
}

static Real SAH(KDtreeNode *tr , int axis , Real plane , 
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

static void findSplitPlane(KDtreeNode *tr)
{
	Real cost = inf;
	tr->axis = KDtreeNode::No_axis;
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
				if (axis == 0)
					tr->axis = KDtreeNode::X_axis;
				else if (axis == 1)
					tr->axis = KDtreeNode::Y_axis;
				else if (axis == 2)
					tr->axis = KDtreeNode::Z_axis;
			}
			nl += p_start;
		}
	}
}

void KDtree::buildTree(KDtreeNode *tr , int dep)
{
	if (dep > depMax)
		return;
	if (tr->objNum <= 1)
		return;

	findSplitPlane(tr);

	tr->div = new DivType[tr->objNum];
	tr->left = new KDtreeNode;
	tr->right = new KDtreeNode;
	KDtreeNode *l , *r;
	l = tr->left;
	r = tr->right;

	int axis;
	if (tr->axis == KDtreeNode::X_axis)
		axis = 0;
	else if (tr->axis == KDtreeNode::Y_axis)
		axis = 1;
	else if (tr->axis == KDtreeNode::Z_axis)
		axis = 2;

	int nl , nr , nb;
	nl = nr = nb = 0;
	for (int i = 0; i < tr->objNum; i++)
	{
		Real st = get_projection(tr->objlist[i]->box.l , axis);
		Real ed = get_projection(tr->objlist[i]->box.r , axis);
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
	l->axis = KDtreeNode::No_axis;

	r->objNum = nb + nr;
	r->objlist = new Geometry*[r->objNum];
	r->left = r->right = NULL;
	r->axis = KDtreeNode::No_axis;

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

	for (int i = 0; i < 3; i++) delete[] tr->e[i];
	delete[] tr->div;
	delete[] index_to_l;
	delete[] index_to_r;

	buildTree(l , dep + 1);
	buildTree(r , dep + 1);
}

Geometry* KDtree::traverse(const Ray& ray , KDtreeNode *tr)
{
	if (tr == NULL)
		return NULL;
	if (tr->axis == KDtreeNode::No_axis)
	{
		Real tmp = inf;
		Intersection inter;
		inter.t = inf;
		Geometry* res = NULL;
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
		return res;
	}
	Real diff = tr->splitPlane - get_projection(ray.origin , (int)tr->axis);
	Real t = diff / get_projection(ray.dir , (int)tr->axis);
	Real a , b;
	if (!tr->box.hit(ray , a , b))
		return NULL;

	KDtreeNode *near , *far;
	if (cmp(diff) == 0)
	{
		Vector3 n;
		if (tr->axis == KDtreeNode::X_axis)
			n = Vector3(1.0 , 0.0 , 0.0);
		else if (tr->axis == KDtreeNode::Y_axis)
			n = Vector3(0.0 , 1.0 , 0.0);
		else if (tr->axis == KDtreeNode::Z_axis)
			n = Vector3(0.0 , 0.0 , 1.0);
		if (cmp(ray.dir ^ n) > 0)
		{
			near = tr->right;
			far = tr->left;
		}
		else
		{
			near = tr->left;
			far = tr->right;
		}
	}
	else if (cmp(diff) > 0)
	{
		near = tr->left;
		far = tr->right;
	}
	else
	{
		near = tr->right;
		far = tr->left;
	}

	if (cmp(t - b) > 0 || cmp(t) < 0)
	{
		return traverse(ray , near);
	}
	else
	{
		if (cmp(t - a) < 0)
		{
			return traverse(ray , far);
		}
		else
		{
			Geometry* res = traverse(ray , near);
			if (res != NULL)
				return res;
			else 
				return traverse(ray , far);
		}
	}
}

void print_tree(FILE *fp , KDtreeNode *tr)
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

	fprintf(fp , "\nSplit plane: axis = %d , pos = %.3lf\n" ,
		(int)tr->axis , tr->splitPlane);

	print_tree(fp , tr->left);
	print_tree(fp , tr->right);
}
