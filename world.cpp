#include "world.h"
#include "point.h"

#include <stdio.h>
#include <string>
#include <ctype.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>

using std::string;
using std::cerr;
using std::endl;
using std::stable_sort;
using std::copy;

void World::readFile(FILE * fp)
{
  const size_t BUFFERSIZE = 4096;
  char buffer[BUFFERSIZE];

  int shapeno = 0;
  int vertexno = 0;
  int shapestart = 0;
  enum { XCOORD, YCOORD, CRUFT };

  int state = XCOORD;
  int newlines(0);
  bool last_line = false;
  char lastc = 0;
  
  string num = "";
  
  for(size_t buffer_start = 0;;buffer_start += BUFFERSIZE)
  {
    fseek(fp, buffer_start, SEEK_SET);
    size_t buffer_size = fread(buffer,sizeof(char), BUFFERSIZE, fp);

    for(size_t p = 0; p <= buffer_size; ++p)
    {
      char c = p == buffer_size ? 0 : buffer[p];
      
      bool is_whitespace = c <= 32;
      bool is_num = isdigit(c);
      bool is_line = c == '\n' || c == '\r';
      
      if (is_num)
      {
        num += c;
        newlines = 0;
      }
      else
      {
        if (num.length() > 0) // new field found
        {
          if (newlines > 0)
          {
            ++vertexno;  
            state = XCOORD;
            vertices[vertexno] = Vertex(0,0,shapeno);
            if (newlines > 1)
            {
              ++shapeno;
              shapes[shapeno] = Shape(vertexno,0);
            }
          }
         
          int i = atoi(num.c_str());
          
          if (state == XCOORD)
            vertices[vertexno].x = i;
          else if (state == YCOORD)
          {
            vertices[vertexno].y = i;
            ++shapes[shapeno].vertices;
          }
          else
            cerr << "Cruft '" << num << "' found." << endl;
          
          if (state < CRUFT) ++state;
          num = "";
        } // num.length() > 0
        
        if (is_line)
        {
          if(!last_line || lastc == c)
            ++newlines; 
          else
            c = lastc;
        }
      }
      
      lastc = c; last_line = is_line;
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
  
  size_t vertexno = 0;
  
  // temporary storage for grown shapes  
  vector<Shape> nshapes;
  vector<GVertex> nvertices;  
  
  // scratch variables
  vector<WPoint> points;
  vector<WPoint> hull;
  
  typedef vector<Shape>::iterator ishape;
  typedef vector<WPoint>::iterator ipoint;
  typedef vector<GVertex>::iterator igvertex;
  
  // rreference points to the robot's reference point
  WPoint * rreference = &robot[DIM(robot)-1];

  --rreference;
  
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
        points.push_back(vertices[vindex] + *rvertex - *rreference);
    }

    // take the outermost shape made from these points
    convexHull(points, hull);
    
    // store the results
    nshapes.push_back(Shape(vertexno,hull.size()));
    for(ipoint i = hull.begin(); i != hull.end(); ++i)
    {
      nvertices[vertexno] = GVertex(Vertex(*i),vertexno);
      ++vertexno;
    }
  }
  
  // now look for overlapping shapes, if two shapes overlap, they need to be merged
  vector<bool> needsmerge(shapes.size());

  for(ishape shape1 = nshapes.begin(); shape1 != nshapes.end(); ++shape1) 
  {
    int sv1 = shape1->startidx;     // start vertex
    int ev1 = sv1 + shape1->vertices; // end vertex
    
    for(ishape shape2 = nshapes.begin(); shape2 != shape1; ++shape2) 
    {

      int sv1 = shape1->startidx; // start vertex
      int nv1 = shape1->vertices; // number of vertices
      int ev1 = sv1 + nv1;

      int sv2 = shape2->startidx; // start vertex
      int nv2 = shape2->vertices; // number of vertices
      int ev2 = sv2 + nv2;
      
      for(int e1 = sv1; e1 < ev1; ++e1) // for each edge of shape1
      {
        GVertex & P = nvertices[e1];
        GVertex & Q = nvertices[(e1+1)%nv1];

        for(int e2 = sv2; e2 < ev2; ++e2) // for each edge of shape2
        {
          GVertex & R = nvertices[e2];
          GVertex & S = nvertices[(e2+1)%nv1];
          
          if (linesIntersect(P,Q,R,S)) // is this result worth caching somewhere?
          {
            shape2->vertices = 0;
            needsmerge[R.shapeno] = true;
            for(int v = sv2; v < ev2; ++v) // for each vertex of shape2
              nvertices[v].shapeno = P.shapeno;
            goto intersection_found;
          }
        }        
      }
    }
    intersection_found:
    continue;
  }
  
  class lessthan // comparison functor
  {
  public:
    operator()(GVertex a, GVertex b)
    {
      return a.shapeno < b.shapeno;
    }
  };

  stable_sort(nvertices.begin(),nvertices.end(),lessthan());

  gshapes.resize(0);
  gvertices.resize(0);

  
  int lastshape = 0;
  int shape = 0;
  igvertex lastv = nvertices.begin();
  for(igvertex v = nvertices.begin();; ++v)
  {
    bool done = v == nvertices.end();
    
    if (done || v->shapeno != lastshape)
    {
      points.resize(0);
      ipoint p = points.begin();
      copy(lastv,v,p);
      convexHull(points,hull);
      
      // store the results
      gshapes[shape] = Shape(vertexno,hull.size());
      for(ipoint i = hull.begin(); i != hull.end(); ++i)
      {
        gvertices[vertexno] = GVertex(Vertex(*i),vertexno);
        ++vertexno;
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

void main()
{
  World w;
}