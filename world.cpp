#include "world.h"
#include "point.h"

#include <stdio.h>
#include <string>
#include <ctype.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <assert.h>
#include <iomanip>
#include <cfloat>
#include <sstream>

using std::cout;

using std::string;
using std::setw;
using std::endl;
using std::stable_sort;
using std::make_heap;
using std::pop_heap;
using std::copy;
using std::reverse;
using std::stringstream;

// the first four points are the vertices of the robot.
// the last point is the robot's reference point (center)
// These were the dimensions given, but the pioneer robot in the saphira module is larger

World::WPoint World::robot[] = 
{
  WPoint(0,0),
  WPoint(370,0),
  WPoint(370,550),
  WPoint(0,550),
  WPoint(185, 275) 
};

// TODO: some repetetive code for loading hulls into point and shape
// arrays can be factored out into a seperate method or class, or possibly
// built into the hull function itself. avoid unneccessary bookeeping by 
// making it more generic

/* returns grown version of h, keeping h intact */
/* pre: h is a convex polygon in counterclockwise order */
void World::fgrowShapes(double amount)
{
  assert(amount > 0);
  
  // copy and reorient original shapes
  
  typedef vector<Vertex>::iterator vi;
  gshapes = shapes;
  gvertices.resize(vertices.size());
  int lastshape = 0;
  vi lasti = vertices.begin();
  for(vi i = lasti; ; ++i)
  {
    bool done = i == vertices.end();
    
    if (done || i->shapeno !=lastshape)
    {
      orient_poly<Vertex,vi>(lasti,i);
      lasti = i;
    }
    if (done) break;
    
    lastshape = i->shapeno;
    int n = i - vertices.begin();
    gvertices[n] = GVertex(*i,n);
  }
  
  for(vector<Shape>::const_iterator shape = shapes.begin(); shape != shapes.end(); ++shape) // for each shape
  {
    int sv = shape->startidx; // start vertex
    int nv = shape->vertices; // number of vertices
    int ev = sv + nv;
      
    /* error checking not fully implemented */
    /*  int done; */
    double j, k;
    WPoint mCA, mAB, mCE, mBD; /* treated like vectors */
    WPoint D, E; /* treated like points */
    
    for(int e = sv; e < ev; ++e) // for each edge of shape
    {
      Vertex & prev = vertices[(e-sv-1+nv) % nv + sv];
      Vertex & p = vertices[e];
      Vertex & next = vertices[(e-sv+1)%nv + sv];
      GVertex & q = gvertices[e];
      
      mCA = p - prev; 
      mAB = next - p;
      mCE = mCA.rot_by_90();
      mBD = mAB.rot_by_90();
      
      mCE.diametrize(amount);
      mBD.diametrize(amount);
      E = prev + mCE;
      D = next + mBD;
      
      /* NOT CHECKING FOR pathological (i.e., non-convex) cases */
      /* this means that we assume that the lines intersect. */
    
      /* the point we want (A') is the intersection of lines */
      
      /* (1) <Dx, Dy> + k <mABx, mABy>
       *  and
       * (2) <Ex, Ey> + j <mCAx, mCAy>
      
       * for the point of intersection:
       * x-coord = Dx + k (mABx) = Ex + j (mCAx)
       * y-coord = Dy + k (mABy) = Ey + j (mCAy)
       * for unique (k, j)
       */
      
      j= (D.x * mAB.y - D.y * mAB.x - E.x * mAB.y + E.y * mAB.x) /
        (mCA.x * mAB.y - mCA.y * mAB.x);
      
      if(mAB.y != 0.0) {
        k = (E.y + j * mCA.y - D.y)/mAB.y;
      } else if(mAB.x != 0.0) {
        k = (E.x + j * mCA.x - D.x)/mAB.x;
      } else {
        /* oh well.  so much for not checking. */
        stringstream msg;
        msg << "big bug: two adjacent vertices coincide! (" << p.x << "," << p.y << ")";
        BARF(msg.str());
      }
      
      q.x = D.x + k * mAB.x;
      q.y = D.y + k * mAB.y;
      /* done calculating q->pt.x and q->pt.y */
    }    
  }
};

void World::growShapes(double mult = 1.0)
{
  // Uses the algorithm in appendix A of the first reference cited in lozano.ps
  // Grows each obstacle by the shape of the robot, but does not take into
  // account robot rotation.

  // m multiplies the size of the robot before growing it into each of the shapes
  // this seems to be neccessary because the provided dimensions are a bit
  // to small to work for the saphira simulator
  
  // After the shapes are grown, this code checks to see if they overlap.
  // If two grown shapes overlap, they are merged into one.
  
  // temporary storage for grown shapes  
  vector<Shape> nshapes;
  vector<GVertex> nvertices;  
  size_t vertexno = 0;  

  // scratch variables
  vector<GVertex> points;
  vector<GVertex> hull;
  
  typedef vector<Shape>::const_iterator ishape;
  typedef vector<WPoint>::const_iterator ipoint;
  typedef vector<GVertex>::const_iterator ivertex;
  typedef vector<GVertex>::const_iterator igvertex;
  
  vector<WPoint> mrobot;
  for(int rv = 0; rv < DIM(robot); ++rv)
    mrobot.push_back(WPoint(robot[rv].x * mult, robot[rv].y * mult));

  // rreference points to the robot's reference point
  WPoint & rreference = mrobot.back();

  // grow each shape   
  for(ishape shape = shapes.begin(); shape != shapes.end(); ++shape) // for each shape
  {
    points.resize(0); // empty points array
    
    for(int v = shape->startidx; v < shape->startidx +shape->vertices; ++v) // for each vertex
    {
      // find the robot's reference positions when each of its vertices touches the obstacle vertex
      // load these positions into the "points" array
      for(int rv = 0; rv < mrobot.size()-1; ++rv) // for each robot vertex
      {
        GVertex p(vertices[v] + mrobot[rv] - rreference, shape-shapes.begin(), v);
        points.push_back(p);
      }
    }

    // take the outermost shape made from these points
    hull.resize(points.size());
    vector<GVertex>::iterator hb = hull.begin(); // need a non constant iterator
    igvertex hullend = convexHull(points.begin(), points.end(), hb);
    
    // store the results
    nshapes.push_back(Shape(vertexno,hullend - hull.begin()));
    for(igvertex i = hull.begin(); i != hullend; ++i)
    {
      nvertices.push_back(*i);
      ++vertexno;
    }
  }
  
  /*
  
  don't use this after all...
  
  vector<Shape> *s1(&nshapes), *s2(&gshapes);
  vector<GVertex> *v1(&nvertices), *v2(&gvertices);  
  
  while(noIntersect(*s1, *v1, *s2, *v2))
  {
    swap(s1,s2);
    swap(v1,v2);
  };
  
  gshapes = *s1;
  gvertices = *v1;
  
  */
  
  gshapes = nshapes;
  gvertices = nvertices;
  
};


class _World_noIntersect_lessthan // comparison functor, can't be declared locally with G++
{
public:
  bool operator()(World::GVertex a, World::GVertex b)
  {
    return a.shapeno < b.shapeno;
  }
};

bool World::noIntersect
(
  vector<Shape> & sbefore, vector<GVertex> & vbefore,
  vector<Shape> & safter, vector<GVertex> & vafter
)
{
  typedef _World_noIntersect_lessthan lessthan;
  typedef vector<Shape>::const_iterator ishape;
  typedef vector<WPoint>::const_iterator ipoint;
  typedef vector<GVertex>::const_iterator ivertex;
  typedef vector<GVertex>::const_iterator igvertex;
  
  bool mergeany = false;  

  // look for overlapping shapes, if two shapes overlap, they need to be merged
  
  // store the new vertex counts in an array
  vector<int> vertexCount;
  for(ishape shape1 = sbefore.begin(); shape1 != sbefore.end(); ++shape1) // for each shape, shape1
  {
    int sv1 = shape1->startidx; // start vertex
    int nv1 = shape1->vertices; // number of vertices
    int ev1 = sv1 + nv1;

    vertexCount.push_back(nv1);
    
    for(ishape shape2 = sbefore.begin(); shape2 != shape1; ++shape2) // for each earlier shape, shape2
    {
      int sv2 = shape2->startidx; // start vertex
      int nv2 = shape2->vertices; // number of vertices
      int ev2 = sv2 + nv2;
      
      for(int e1 = sv1; e1 < ev1; ++e1) // for each edge of shape1
      {
        GVertex & P = vbefore[e1];
        GVertex & Q = vbefore[(e1-sv1+1)%nv1 + sv1];

        for(int e2 = sv2; e2 < ev2; ++e2) // for each edge of shape2
        {
          GVertex & R = vbefore[e2];
          GVertex & S = vbefore[(e2-sv2+1)%nv2 + sv2];
          
          if (linesIntersect(P,Q,R,S)) // if edges for these shapes intersect
          {
            mergeany = true;
            int latershape = P.shapeno;
            int earliershape = R.shapeno;
            // subsume the earlier shape into the later one
            vertexCount[latershape] += vertexCount[earliershape];
            vertexCount[earliershape] = 0;
            for(int v = 0; v < sv1; ++v) // subsume all vertexes which are part of this shape
              if(vbefore[v].shapeno == earliershape)
                vbefore[v].shapeno = latershape;
            goto intersection_found;
          }
        }        
      }
      intersection_found:
      continue; // examine the next shape2
    } // for shape2
  } // for shape1
  
  if (!mergeany) return false;
  
  // sort vertices by shape number. ordering of vertices within each shape is preserved
  stable_sort(vbefore.begin(),vbefore.end(),lessthan());

  // clear out the existing grown shape data
  safter.resize(100);
  safter.resize(0);
  vafter.resize(100);
  vafter.resize(0);
  
  // temporary place to store merged hulls
  vector<GVertex> ghull;
  
  int lastshape = vbefore.front().shapeno;
  int shape = 0;
  igvertex lastv = vbefore.begin();
  for(igvertex v = vbefore.begin(); ; ++v) // for each vertex
  {
    bool done = v == vbefore.end();
    
    if (done || v->shapeno != lastshape) // new shape
    {
      assert(v - lastv == vertexCount[lastshape]); // sanity check

      igvertex gvstart, gvend;
      vector<GVertex>::iterator hullstart, hullend;
      
      if (vertexCount[lastshape] > sbefore[lastshape].vertices) // shape is merged
      {
        ghull.resize(v - lastv);
        hullstart = ghull.begin();
        hullend = convexHull(lastv, v, hullstart);
        gvstart = hullstart;
        gvend = hullend;
      }
      else if(vertexCount[lastshape] == sbefore[lastshape].vertices) // shape is unchanged
      {
        gvstart = lastv;
        gvend = v;
      }
      else
      {
        assert(false);
      }

      // add the shape and copy the vertices (from either a hull or the existing vertex array)
      safter.push_back(Shape(vafter.size(),gvend - gvstart));
      for(igvertex i = gvstart; i != gvend; ++i)
      {
        vafter.push_back(*i);
        vafter.back().shapeno = shape;
      }
      
      ++shape;
      lastv = v;
    }
    
    if (done) break;
    
    lastshape = v->shapeno;
  }
  return true;
};

World::GVertex const & World::get_node(int i)
{
  int n = nodes[i];
  if (n == START)
    return start;
  else if (n == GOAL)
    return goal;
  else
    return gvertices[n];
};

void World::makeVisibility()
{
  typedef vector<GVertex>::const_iterator ivertex;
  typedef vector<Shape>::const_iterator ishape;
  typedef vector<WPoint>::const_iterator ipoint;
  
  // find center start and goal points
  for(ipoint v = startarea.begin(); v != startarea.end(); ++v)
    start = start + *v;
  start = start / (startarea.end() - startarea.begin());

  for(ipoint v = goalarea.begin(); v != goalarea.end(); ++v)
    goal = goal + *v;
  goal = goal / (goalarea.end() - goalarea.begin());


  // fill up nodes array, include start and goal points,
  // exclude any points that are inside obstacles
  nodes.resize(0);
  nodes.push_back(START);  
  
  vector<GVertex> const & vertices = this->gvertices;
  vector<Shape> const & shapes = this->gshapes;

  for(ivertex v = vertices.begin(); v != vertices.end(); ++v)
  {
    bool reject = false;
    for(ishape shape = shapes.begin(); shape != shapes.end(); ++shape)
    {
      int sv = shape->startidx; // start vertex
      int nv = shape->vertices; // number of vertices
      int ev = sv + nv;
      bool inside = true;
      for(int e = sv; e < ev; ++e) // for each edge of shape1
      {
        GVertex const & P = vertices[e];
        GVertex const & Q = vertices[(e-sv+1)%nv + sv];

        // assumes we are traversing the vertices of the polygon in
        // ccw order (as outputted by the convex hull algorithm)
        if (v->line_rside(P, Q) != WPoint::LEFT_SIDE)
        {
          inside = false;
          break;
        }
      };
      if (inside)
      {
        reject = true;
        break;
      }
    }
    if (!reject)
    {
      nodes.push_back(v - vertices.begin());
    }
  }  
  nodes.push_back(GOAL);

  int gpl = nodes.size();
 
  isvisible.resize(gpl);
  distanceCache.resize(gpl);

  for(int p = 0; p < gpl; ++p) // try each potential visiblity graph edge
  for(int q = 0; q <= p; ++q)
  {
    GVertex const & P = get_node(p);
    GVertex const & Q = get_node(q);

    bool visible = true;

    if (P.shapeno == Q.shapeno)
    {
      if(P.shapeno >= 0)
      {
        int nv = shapes[P.shapeno].vertices;
        visible = (p - q == 1 || p - q == nv - 1);
      }
      else
        visible = false;
    }
    else
    {
      for(ishape shape = shapes.begin(); shape != shapes.end(); ++shape) 
      {
        int sv = shape->startidx;   // start vertex
        int nv = shape->vertices;    // number of vertices
        int ev = sv + nv;

        for(int e = sv; e < ev; ++e) // for each edge of shape
        {
          GVertex const & R = vertices[e];
          GVertex const & S = vertices[(e-sv+1)%nv + sv];
          if (linesIntersect(P,Q,R,S))
          {
            visible = false;
            goto skip;
          }
        }        
      }
    }
    skip:
    isvisible(p,q) = visible;
    if (visible)
      distanceCache(p,q) = visible ? P.distanceTo(Q) : DBL_MAX;
  }
}  


struct _World_findPath_IntDist // metrowerks won't allow this to be instantiated if it is declared inside the function
{
  double d;
  int i;
  vector<int>::iterator heappos;
  _World_findPath_IntDist() : i(-1), d(HUGE_VAL) {}
};


// heap comparison functor, can't be declared locally with g++
class _World_findPath_pcompare
{
public:
  typedef _World_findPath_IntDist IntDist;
  vector<IntDist> & d;
  _World_findPath_pcompare(vector<IntDist> & d_) : d(d_) { }
  bool operator()(int a, int b)
  {
    return d[a].d > d[b].d; // priority_queue puts max first
  }
};

class _World_findPath_updatepos
{
private:  
  typedef _World_findPath_IntDist IntDist;
  vector<IntDist> & _d;

public:  
  _World_findPath_updatepos(vector<IntDist> & d) : _d(d) {}
  
  void operator()(int i, vector<int>::iterator newpos)
  {
    _d[i].heappos = newpos;
  }
};

void World::findPath()
{
  typedef _World_findPath_pcompare pcompare;
  typedef _World_findPath_IntDist IntDist;
  typedef vector<int>::iterator vi;
  typedef _World_findPath_updatepos updatepos;
  vector<IntDist> d(nodes.size());
  d[0].d = 0; // start node;

  vector<int> heap;
  for(int i = 0; i < nodes.size(); ++i)
    heap.push_back(i);
  make_heap(heap.begin(),heap.end(),pcompare(d));
  for(vi i = heap.begin(); i != heap.end(); ++i) // store heap ordering
    d[*i].heappos = i;

  vi endheap = heap.end();
  while(endheap > heap.begin()) 
  {
    int v = heap[0];
    pop_mheap(heap.begin(), endheap, pcompare(d), updatepos(d));
    --endheap;
    
    IntDist & V = d[v];

    for(int w = 0; w < nodes.size(); ++w)
    if (isvisible(v,w))
    {
      IntDist & W = d[w];
      double Wdistance = V.d + distanceCache(v,w);
      if (Wdistance < W.d)
      {
        W.d = Wdistance;
        W.i = v;
        decreasekey_mheap(heap.begin(), W.heappos, pcompare(d), updatepos(d));
      };  
    }
  }
  
  path.resize(0); // clear the existing path
  int i = nodes.size() - 1; // end point
  for(;;)
  {
    path.push_back(i);
    int next = d[i].i;
    if (next < 0) break;
    i = next;
  }
}

void World::reorient()
{
  typedef vector<Shape>::const_iterator ishape;
  vector<GVertex> const & vertices = this->gvertices;
  vector<Shape> const & shapes = this->gshapes;
  
  int p = 0;
  for(int q = 1; q < nodes.size(); ++q)
  {
    GVertex const & P = get_node(p);
    GVertex const & Q = get_node(q);

    bool visible = true;

    for(ishape shape = shapes.begin(); shape != shapes.end(); ++shape) 
    {
      int sv = shape->startidx;   // start vertex
      int nv = shape->vertices;    // number of vertices
      int ev = sv + nv;

      for(int e = sv; e < ev; ++e) // for each edge of shape
      {
        GVertex const & R = vertices[e];
        GVertex const & S = vertices[(e-sv+1)%nv + sv];
        if (linesIntersect(P,Q,R,S))
        {
          visible = false;
          goto skip;
        }
      }        
    }
    skip:
    isvisible(p,q) = visible;
    if (visible)
      distanceCache(p,q) = visible ? P.distanceTo(Q) : DBL_MAX;
  }
}  
  
void World::reorient(WPoint newobstacle, coord radius)
{
  for(int i = 0; i < isvisible.size(); ++i)
  for(int j = 0; j < i; ++j)
  if (isvisible(i,j))
  {
    GVertex I = get_node(i);
    GVertex J = get_node(j);
    if (newobstacle.distanceTo(I,J) < radius)
      isvisible(i,j) = false;
  }
  
  reorient();
};

void World::outputVisibility(FILE * fp)
{
  for(int i = 0; i < isvisible.size(); ++i)
  for(int j = 0; j < i; ++j)
  {
    if (isvisible(i,j))
    {
      GVertex I = get_node(i);
      GVertex J = get_node(j);
      fprintf(fp, "%i %i # shape %i, vertex %i\n"  , I.x, I.y, I.shapeno, nodes[i]);
      fprintf(fp, "%i %i # shape %i, vertex %i\n\n", J.x, J.y, J.shapeno, nodes[j]);
    }
  }
}

void World::outputPath(FILE * fp)
{
  for(vector<int>::const_iterator p = path.begin(); p != path.end(); ++p)
  {
    GVertex g = get_node(*p);
    fprintf(fp, "%i %i\n",g.x,g.y);
  }
  fprintf(fp, "\n");
}

void World::outputTargets(FILE * fp)
{
  typedef vector<WPoint>::const_iterator ivertex;
  
  for(ivertex i = goalarea.begin(); ; ++i)
  {
    if (i == goalarea.end())
    {
      fprintf(fp, "%i %i\n\n",goalarea[0].x,goalarea[0].y);
      break;
    }
    else
      fprintf(fp, "%i %i\n",i->x,i->y);
  }
  
  for(ivertex i = startarea.begin(); ; ++i)
  {
    if (i == startarea.end())
    {
      fprintf(fp, "%i %i\n\n",startarea[0].x,startarea[0].y);
      break;
    }
    else
      fprintf(fp, "%i %i\n",i->x,i->y);
  }
};


void World::describe(bool show_vertices, bool show_gvertices, bool show_nodes, bool show_visibility)
{
  typedef vector<Shape>::const_iterator ishape;
  typedef vector<Vertex>::const_iterator ivertex;
  typedef vector<GVertex>::const_iterator igvertex;
  
  if(show_vertices)
  {
    cerr << "Printing shapes table" << endl;
    cerr << "i\tstart\tvertices\t" << endl;
    for(ishape shape = shapes.begin(); shape != shapes.end(); ++shape)
    {
      cerr << (shape - shapes.begin()) << '\t' << shape->startidx << '\t'
           << shape->vertices << endl;
    }
    
    cerr << "Printing vertex table" << endl;
    cerr << "i\tx\ty\tshapeno" << endl;
    for(ivertex vertex = vertices.begin(); vertex != vertices.end(); ++vertex)
    {
      cerr << (vertex - vertices.begin()) << '\t' << vertex->x << '\t'
           << vertex->y << '\t' << vertex->shapeno << endl;
    }
  }; 

  if(show_gvertices)
  {
    cerr << "Printing gshapes table" << endl;
    cerr << "i\tstart\tvertices\t" << endl;
    for(ishape shape = gshapes.begin(); shape != gshapes.end(); ++shape)
    {
      cerr << (shape - gshapes.begin()) << '\t' << shape->startidx << '\t'
           << shape->vertices <<endl;
    }
    
    cerr << "Printing vertex table" << endl;
    cerr << "i\tx\ty\tshapeno\told vertex no" << endl;
    for(igvertex vertex = gvertices.begin(); vertex != gvertices.end(); ++vertex)
    {
      cerr << (vertex - gvertices.begin()) << '\t' << vertex->x << '\t'
           << vertex->y << '\t' << vertex->shapeno << '\t' << vertex->vertexno
           << endl;
    };
  };
  
  if (show_nodes)
  {
    int s = nodes.size();
    cerr << "#" << '\t' << "x" << '\t' << "y" << '\t' << "gvertex#" << endl;    
    for(int i = 0; i < s; ++i)
    {
      GVertex const & gv = get_node(i);
      cerr << i << '\t' << gv.x << '\t' << gv.y << '\t';
      if(nodes[i] == START)
        cerr << "startnode";
      else if (nodes[i] == GOAL)
        cerr << "goalnode";
      else
        cerr << nodes[i];
      cerr << endl;  
    };
  };
  
  if (show_visibility)
  {
    cerr << "   ";
    for(int i = 0; i < isvisible.size(); ++i)    
      cerr << setw (3) << i;
    cerr << endl;

    for(int i = 0; i < isvisible.size(); ++i)
    {
      cerr << setw(3) << i;
      for(int j = 0; j < isvisible.size(); ++j)
        cerr << setw (3) << (isvisible(j,i) ? 'T' : 'F');
      cerr << endl;
    };
  };
}
