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

template<typename PointType>
void World::readFile(FILE * fp, vector<PointType> & vertices, vector<Shape> * shapes)
{
  // clear out existing data
  if (shapes) shapes->resize(0);
  vertices.resize(0);

  // state variables
  int shapeno = 0;
  int vertexno = 0;
  int lastvertexno = 0;
  Vertex vertex(0,0,shapeno);

  enum { XCOORD, YCOORD, CRUFT };
  int state = XCOORD;

  int newlines(0);
  int lineno = 1;
  string num = "";

  const size_t BUFFERSIZE = 4096;
  size_t buffer_size;
  char buffer[BUFFERSIZE];
  for(;;)
  {
    buffer_size = fread(buffer,sizeof(char), BUFFERSIZE, fp);

    for(size_t p = 0; p <= buffer_size; ++p)
    {
      char c;

      if (buffer_size == 0)
      {
        newlines = 2;
        c = 0;
      }
      else
        c = buffer[p];
      
      bool is_whitespace = c <= 32;
      bool is_num = isdigit(c) || c == '-';
      bool is_line = c == '\n';
      
      if (is_num)
        num += c;
      else
      {
        if (num.length() > 0 || buffer_size == 0) // new field found
        {
          if (newlines > 0)
          {
            if (vertexno != 0 || state == CRUFT) // ignore leading newlines
            {  
              if (state < CRUFT)
                cerr << "Parse error on line " << lineno << ". Vertex " << vertexno << " has invalid x or y coordinates" << endl;
              state = XCOORD;

              // if the last vertex is the same as the first one, don't add it
              if (newlines <= 1 || !vertex.equals(vertices[lastvertexno]))
              {
                vertices.push_back(vertex);
                ++vertexno;
              }

              if (newlines > 1)
              {
                if (shapes) shapes->push_back(Shape(lastvertexno,vertexno - lastvertexno));
                ++shapeno;
                vertex.shapeno = shapeno;
                lastvertexno = vertexno;
              }
            }  
            newlines = 0;
          }
         
          if (state == XCOORD)
            vertex.x = atoi(num.c_str());
          else if (state == YCOORD)
            vertex.y = atoi(num.c_str());
          else
            cerr << "Cruft '" << num << "' found on line " << lineno << "." << endl;
          
          if (state < CRUFT) ++state;
          num = "";
        } // num.length() > 0
        
        if (is_line) { ++newlines; ++lineno; }
      }
    } // for p
    if (buffer_size == 0) break;
  }
}

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
  if (i == START)
    return start;
  else if (i == GOAL)
    return goal;
  else
    return gvertices[i];
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
      nodes.push_back(v - gvertices.begin());
    }
  }  
  nodes.push_back(GOAL);
  


  int gpl = nodes.size();
  
  for(int p = 0; p < gpl; ++p) // try each potential visiblity graph edge
  for(int q = 0; q < p; ++q)
  {
    GVertex & P = get_node(p);
    GVertex & Q = get_node(q);
    if (P.shapeno == Q.shapeno && P.shapeno >= 0)
    {
      int nv = shapes[P.shapeno].vertices;
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

template<class InputIterator>
void World::outputShapes(FILE * fp, InputIterator istart, InputIterator iend)
{
  if (istart == iend) return;
  
  int lastshape = istart->shapeno;
  InputIterator firsti = istart;
  
  cerr << "i\tx\ty\tshapeno" << endl;
  for(InputIterator i = istart;; ++i)
  {
    bool over = i == iend;
    
    if (over || lastshape != i->shapeno)
    {
      fprintf(fp, "%i %i\n\n",firsti->x,firsti->y);
      firsti = i;
    }
   
    if (over) break;
    
    fprintf(fp, "%i %i\n",i->x,i->y);
    
    lastshape = i->shapeno;
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
/*
void main()
{
  World w;
  FILE * fp;
  
  fp = fopen("M:/russ/My Documents/quickman/obstacle.txt","r");
  w.readFile(fp,w.vertices,&w.shapes);
  fclose(fp);

  fp = fopen("M:/russ/My Documents/quickman/start.txt","r");
  w.readFile(fp,w.startarea);
  fclose(fp);

  fp = fopen("M:/russ/My Documents/quickman/goal.txt","r");
  w.readFile(fp,w.goalarea);
  fclose(fp);


  w.growShapes();
  w.describe();
  
  fp = fopen("M:/russ/My Documents/quickman/rgrow.txt","w");
  w.outputShapes(fp, w.vertices.begin(), w.vertices.end());
  w.outputShapes(fp, w.gvertices.begin(), w.gvertices.end());
  fclose(fp);
}
*/