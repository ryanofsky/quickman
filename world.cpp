#include "world.h"
#include "point.h"

#include <stdio.h>
#include <string>
#include <ctype.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <assert.h>

using std::string;
using std::cerr;
using std::endl;
using std::stable_sort;
using std::make_heap;
using std::pop_heap;
using std::copy;
using std::reverse;

// the first four points are the vertices of the robot.
// the last point is the robot's reference point (center)
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

void World::growShapes()
{
  // Uses the algorithm in appendix A of the first reference cited in lozano.ps
  // Grows each obstacle by the shape of the robot, but does not take into
  // account robot rotation.
  
  // After the shapes are grown, this code checks to see if they overlap.
  // If two grown shapes overlap, they are merged into one.
  
  // temporary storage for grown shapes  
  vector<Shape> nshapes;
  vector<GVertex> nvertices;  
  size_t vertexno = 0;  

  // scratch variables
  vector<GVertex> points;
  vector<GVertex> hull;
  
  typedef vector<Shape>::iterator ishape;
  typedef vector<WPoint>::iterator ipoint;
  typedef vector<GVertex>::iterator ivertex;
  typedef vector<GVertex>::iterator igvertex;
  
  // rreference points to the robot's reference point
  WPoint * rreference = &robot[DIM(robot)-1];

  //grow each shape   
  for(ishape shape = shapes.begin(); shape != shapes.end(); ++shape) // for each shape
  {
    points.resize(0); // empty points array
    
    for(int vertex = 0; vertex < shape->vertices; ++vertex) // for each vertex
    {
      int vindex = shape->startidx + vertex;

      // find the robot's reference positions when each of its vertices touches the obstacle vertex
      // load these positions into the "points" array
      for(WPoint * rvertex = &robot[0]; rvertex != rreference; ++rvertex) // for each robot vertex
        points.push_back(GVertex(vertices[vindex] + *rvertex - *rreference,shape-shapes.begin(),vindex));
    }

    // take the outermost shape made from these points
    hull.resize(points.size());
    igvertex hullend = convexHull(points.begin(), points.end(), hull.begin());
    
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
    cerr << "once" << endl;
    swap(s1,s2);
    swap(v1,v2);
  };
  
  gshapes = *s1;
  gvertices = *v1;
  
  */
  
  gshapes = nshapes;
  gvertices = nvertices;
  
};


bool World::noIntersect
(
  vector<Shape> & sbefore, vector<GVertex> & vbefore,
  vector<Shape> & safter, vector<GVertex> & vafter
)
{
  
  typedef vector<Shape>::iterator ishape;
  typedef vector<WPoint>::iterator ipoint;
  typedef vector<GVertex>::iterator ivertex;
  typedef vector<GVertex>::iterator igvertex;
  
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
  
  class lessthan // comparison functor
  {
  public:
    operator()(GVertex a, GVertex b)
    {
      return a.shapeno < b.shapeno;
    }
  };

  // sort vertices by shape number. ordering of vertices within each shape is preserved
  stable_sort(vbefore.begin(),vbefore.end(),lessthan());

  // clear out the existing grown shape data
  safter.resize(0);
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
      
      if (vertexCount[lastshape] > sbefore[lastshape].vertices) // shape is merged
      {
        ghull.resize(v - lastv);
        gvstart = ghull.begin();
        gvend = convexHull(lastv, v, gvstart);
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

World::GVertex & World::get_node(int i)
{
  if (nodes[i] == START)
    return start;
  else if (nodes[i] == GOAL)
    return goal;
  else
  {
    int n = nodes[i];
    World::GVertex y = gvertices[nodes[i]];
    
    return gvertices[nodes[i]];
  }  
    
    
}


void World::makeVisibility()
{
  typedef vector<WPoint>::iterator ipoint;
  typedef vector<GVertex>::iterator ivertex;
  typedef vector<Shape>::iterator ishape;

  for(ipoint v = startarea.begin(); v != startarea.end(); ++v)
    start = start + *v;
  start = start / (startarea.end() - startarea.begin());

  for(ipoint v = goalarea.begin(); v != goalarea.end(); ++v)
    goal = goal + *v;
  goal = goal / (goalarea.end() - goalarea.begin());

  nodes.resize(gvertices.size()+2);
  nodes.resize(0);
  nodes.push_back(START);
  
  typedef vector<GVertex>::iterator ivertex;
  typedef vector<Shape>::iterator ishape;
  
  // fill up nodes array, include start and goal points,
  // exclude any points that are inside obstacles

  for(ivertex v = gvertices.begin(); v != gvertices.end(); ++v)
  {
    double th;  
  
    bool reject = false;
    for(ishape shape = gshapes.begin(); shape != gshapes.end(); ++shape)
    {
      int sv = shape->startidx; // start vertex
      int nv = shape->vertices; // number of vertices
      int ev = sv + nv;
      bool inside = true;
      for(int e = sv; e < ev; ++e) // for each edge of shape1
      {
        GVertex & P = gvertices[e];
        GVertex & Q = gvertices[(e-sv+1)%nv + sv];
        th = atan2(v->y - P.y, v->x - P.x) - atan2(Q.y - P.y, Q.x - P.x);
        if(sin(th) > 0)
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
      if (nodes.size() == 28)
      {
        cerr << "wow";
      }
      int wsd = v - gvertices.begin();
      nodes.push_back(wsd);
    }
  }  
  nodes.push_back(GOAL);
  

  int gpl = nodes.size();
 
  isvisible.resize(gpl);
  distanceCache.resize(gpl);
  
  for(int p = 0; p < gpl; ++p) // try each potential visiblity graph edge
  for(int q = 0; q < p; ++q)
  {
    if (p == 29)
    {
      cerr << "ehllsd";
    }
    
    GVertex & P = get_node(p);
    GVertex & Q = get_node(q);
    if (P.shapeno == Q.shapeno && P.shapeno >= 0)
    {
      int nv = gshapes[P.shapeno].vertices;
      isvisible(p,q) = (p - q== 1 || p - q == gshapes[P.shapeno].vertices - 1);
    }
    else
    {
      bool visible = true;
      // 
      for(ishape shape = gshapes.begin(); shape != gshapes.end(); ++shape) 
      {
        int sv = shape->startidx;   // start vertex
        int nv = shape->vertices;    // number of vertices
        int ev = sv + nv;

        for(int e = sv; e < ev; ++e) // for each edge of shape
        {
          GVertex & R = gvertices[e];
          GVertex & S = gvertices[(e-sv+1)%nv + sv];
          if (linesIntersect(P,Q,R,S))
          if (true)
          {
            visible = false;
            goto notvisible;
          }
        }        
      }
      notvisible:
      isvisible(p,q) = visible;
      
      if (visible)
        distanceCache(p,q) = P.distanceTo(Q);
    }
  }
}  


struct _World_findPath_IntDist // metrowerks won't allow this to be instantiated if it is declared inside the function
{
  double d;
  int i;
  bool visited;
  _World_findPath_IntDist() : i(-1), d(100000) {}
};

void World::findPath()
{
  typedef _World_findPath_IntDist IntDist;
  
  path.clear();

  vector<IntDist> d(nodes.size());
  vector<int> heap;
  for(int i = 0; i < nodes.size(); ++i)
    heap.push_back(i);

  d[0].d = 0; // start node;

  // heap comparison functor
  class pcompare
  {
  public:

    vector<IntDist> & d;
    pcompare(vector<IntDist> & d_) : d(d_) { }
    operator()(int a, int b)
    {
      return d[a].d > d[b].d; // priority_queue puts max first
    }
  };
  
  while(heap.size() > 0) 
  {
    make_heap(heap.begin(),heap.end(),pcompare(d));
    
    int v = heap[0];
    pop_heap(heap.begin(),heap.end(),pcompare(d));
    heap.pop_back();
    
    IntDist & V = d[v];
    V.visited = true;
    
    for(int w = 0; w <= nodes.size(); ++w)
    if (isvisible(v,w))
    {
      IntDist & W = d[w];
      if (!W.visited)
      {
        double Wdistance = V.d + distanceCache(v,w);
        if (Wdistance < W.d)
          W.d = Wdistance;
        W.i = v;  
      }    
    }
  }
  
  int i = d.size() - 1;
  for(;;)
  {
    path.push_back(i);
    int next = d[i].i;
    if (next <= 0) break;
    i = next;
  }
  reverse(path.begin(), path.end());
}

void World::outputVisibility(FILE * fp)
{
  for(int i = 0; i < isvisible.size(); ++i)
  for(int j = 0; j < i; ++j)
  {
    if (isvisible(i,j))
    {
      GVertex I = get_node(i);
      GVertex J = get_node(j);
      fprintf(fp, "%i %i\n%i %i\n\n",I.x,I.y,J.x,J.x);
    }
  }
}

void World::outputPath(FILE * fp)
{
  for(vector<int>::iterator p = path.begin(); p != path.end(); ++p)
  {
    GVertex g = get_node(*p);
    fprintf(fp, "%i %i\n",g.x,g.y);
  }
  fprintf(fp, "\n");
}

void World::outputTargets(FILE * fp)
{
  typedef vector<WPoint>::iterator ivertex;
  
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
}  


void World::describe()
{
  typedef vector<Shape>::const_iterator ishape;
  typedef vector<Vertex>::const_iterator ivertex;
  typedef vector<GVertex>::const_iterator igvertex;
  
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
  
  cerr << "Printing gshapes table" << endl;
  cerr << "i\tstart\tvertices\t" << endl;
  for(ishape shape = gshapes.begin(); shape != gshapes.end(); ++shape)
  {
    cerr << (shape - gshapes.begin()) << '\t' << shape->startidx << '\t'
         << shape->vertices << endl;
  }
  
  cerr << "Printing vertex table" << endl;
  cerr << "i\tx\ty\tshapeno\told vertex no" << endl;
  for(igvertex vertex = gvertices.begin(); vertex != gvertices.end(); ++vertex)
  {
    cerr << (vertex - gvertices.begin()) << '\t' << vertex->x << '\t'
         << vertex->y << '\t' << vertex->shapeno << '\t' << vertex->vertexno
         << endl;
  }  
}
