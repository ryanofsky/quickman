#include "world.h"

#include <stdio.h>
#include <string>
#include <ctype.h>
#include <stdlib.h>
#include <iostream>

using std::string;
using std::cerr;
using std::endl;

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

World::WPoint World::robot[] = 
{
  WPoint(0,0),
  WPoint(370,0),
  WPoint(370,550),
  WPoint(0,550),
  WPoint(185, 275) 
};

void World::makeVisibility()
{
  
  // Step 1: Grow shapes
  
  size_t vertexno = 0;
  
  vector<WPoint> points;
  vector<WPoint> hull;
  
  WPoint rreference = robot[DIM(robot)-1];
   
  for(int shape = 0; shape < shapes.size(); ++shape)
  {
    points.resize(0);
    for(int vertex = 0; vertex < shapes[shape].vertices; ++vertex)
    {
      int vindex = shapes[shape].startidx + vertex;
      for(int rvertex = 0; rvertex < DIM(robot) - 1; ++rvertex)
        points.push_back(points[vertex] + robot[rvertex] - rreference);
    }
    convexHull(points, hull);
    typedef vector<WPoint>::iterator ipoint;
    
    gshapes[shape] = Shape(vertexno,0);
    for(ipoint i = hull.begin(); i != hull.end(); ++i)
    {
      gvertices[vertexno] = GVertex(Vertex(*i),vertexno);
      ++vertexno;
    }
    gshapes[shape].vertices = vertexno - gshapes[shape].startidx;
  }
  
  int gpl = gvertices.size();
  SMatrix<bool> merge(shapes.size());
  SMatrix<bool> gintersect(gvertices.size());
  
  for(int p = 0; p < gpl; ++p)
  for(int q = 0; q < p; ++q)
  {
    GVertex & P = gvertices[p];
    GVertex & Q = gvertices[q];
    if (P.shapeno == Q.shapeno)
    {
      int nv = shapes[P.shapeno].vertices;
      gintersect(p,q) = (p-q == 1 || p-q == nv-1) ? VISIBLE : BLOCKED;
    }
    else
    {
      bool blocked = false;
      for(int r = 0; p < gpl; ++p)
      for(int s = 0; q < p; ++q)
      {
        GVertex & R = gvertices[r];
        GVertex & S = gvertices[s];
        
        if (linesIntersect(P,Q,R,S)) 
        {
          if (P.shapeno == Q.shapeno && R.shapeno == S.shapeno)
            merge(P.shapeno, R.shapeno) = true;
          else
          {
            blocked = true;
            goto intersection_found;
          }
        }
      }
      intersection_found:
      gintersect(p,q) = blocked ? VISIBLE : BLOCKED;
    }
  }
  
  // handle merges...
  
};

void World::findPath()
{
}

void main()
{
  World w;
}