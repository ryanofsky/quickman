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
using std::copy;

void World::readFile(FILE * fp)
{
  // clear out existing data
  shapes.resize(0);
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
                shapes.push_back(Shape(lastvertexno,vertexno - lastvertexno));
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
  
  // now look for overlapping shapes, if two shapes overlap, they need to be merged
  
  // store the new vertex counts in an array
  vector<int> vertexCount;
  for(ishape shape1 = nshapes.begin(); shape1 != nshapes.end(); ++shape1) // for each shape, shape1
  {
    int sv1 = shape1->startidx; // start vertex
    int nv1 = shape1->vertices; // number of vertices
    int ev1 = sv1 + nv1;

    vertexCount.push_back(nv1);
    
    for(ishape shape2 = nshapes.begin(); shape2 != shape1; ++shape2) // for each earlier shape, shape2
    {
      int sv2 = shape2->startidx; // start vertex
      int nv2 = shape2->vertices; // number of vertices
      int ev2 = sv2 + nv2;
      
      for(int e1 = sv1; e1 < ev1; ++e1) // for each edge of shape1
      {
        GVertex & P = nvertices[e1];
        GVertex & Q = nvertices[(e1-sv1+1)%nv1 + sv1];

        for(int e2 = sv2; e2 < ev2; ++e2) // for each edge of shape2
        {
          GVertex & R = nvertices[e2];
          GVertex & S = nvertices[(e2-sv2+1)%nv2 + sv2];
          
          if (linesIntersect(P,Q,R,S)) // if edges for these shapes intersect
          {
            int latershape = P.shapeno;
            int earliershape = R.shapeno;
            // subsume the earlier shape into the later one
            vertexCount[latershape] += vertexCount[earliershape];
            vertexCount[earliershape] = 0;
            for(int v = 0; v < sv1; ++v) // subsume all vertexes which are part of this shape
              if(nvertices[v].shapeno == earliershape)
                nvertices[v].shapeno = latershape;
            goto intersection_found;
          }
        }        
      }
      intersection_found:
      continue; // examine the next shape2
    } // for shape2
  } // for shape1
  
  class lessthan // comparison functor
  {
  public:
    operator()(GVertex a, GVertex b)
    {
      return a.shapeno < b.shapeno;
    }
  };

  // sort vertices by shape number. ordering of vertices within each shape is preserved
  stable_sort(nvertices.begin(),nvertices.end(),lessthan());

  // clear out the existing grown shape data
  gshapes.resize(0);
  gvertices.resize(0);
  
  // temporary place to store merged hulls
  vector<GVertex> ghull;
  
  int lastshape = nvertices.front().shapeno;
  int shape = 0;
  igvertex lastv = nvertices.begin();
  for(igvertex v = nvertices.begin(); ; ++v) // for each vertex
  {
    bool done = v == nvertices.end();
    
    if (done || v->shapeno != lastshape) // new shape
    {
      assert(v - lastv == vertexCount[lastshape]); // sanity check

      igvertex gvstart, gvend;
      
      if (vertexCount[lastshape] > nshapes[lastshape].vertices) // shape is merged
      {
        ghull.resize(v - lastv);
        gvstart = ghull.begin();
        gvend = convexHull(lastv, v, gvstart);
      }
      else if(vertexCount[lastshape] == nshapes[lastshape].vertices) // shape is unchanged
      {
        gvstart = lastv;
        gvend = v;
      }
      else
      {
        assert(false);
      }

      // add the shape and copy the vertices (from either a hull or the existing vertex array)
      gshapes.push_back(Shape(vertexno,gvend - gvstart));
      for(igvertex i = gvstart; i != gvend; ++i)
      {
        gvertices.push_back(*i);
        gvertices.back().shapeno = shape;
      }
      
      ++shape;
      lastv = v;
    }
    
    if (done) break;
    
    lastshape = v->shapeno;
  }
};

  
void World::makeVisibility()
{
  typedef vector<Shape>::iterator ishape;

  int gpl = gvertices.size();
  
  for(int p = 0; p < gpl; ++p)
  for(int q = 0; q < p; ++q)
  {
    GVertex & P = gvertices[p];
    GVertex & Q = gvertices[q];
    if (P.shapeno == Q.shapeno)
    {
      int nv = shapes[P.shapeno].vertices;
      visibility(p,q) = (p-q == 1 || p-q == nv-1);
    }
    else
    {
      bool visible = true;
      for(ishape shape = gshapes.begin(); shape != gshapes.end(); ++shape) 
      {
        int sv = shape->startidx;   // start vertex
        int nv = shape->vertices;    // number of vertices
        int ev = sv + nv;

        for(int e = sv; e < ev; ++e) // for each edge of shape
        {
          GVertex & R = gvertices[e];
          GVertex & S = gvertices[(e+1)%nv];
          
          if (linesIntersect(P,Q,R,S))
          if (true)
          {
            visible = false;
            goto notvisible;
          }
        }        
      }
      notvisible:
      visibility(p,q) = visible;
    }
  }
}  

void World::findPath()
{
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

void main()
{
  World w;
  FILE * fp = fopen("M:/russ/My Documents/quickman/obstacle.txt","r");
  w.readFile(fp);
  fclose(fp);
  w.growShapes();
  w.describe();
}