#ifndef world_h
#define world_h

#include <stdio.h>
#include "point.h"
#include "smatrix.h"

class World
{
public:

  //! grow obstacles 
  void growShapes();

  // generate visibility graph
  void makeVisibility();
  
  //! find the optimal path through the obstacles  
  void findPath();

  typedef int coord;
  typedef Point<coord> WPoint;

  struct Vertex : public WPoint
  {
    int shapeno;
  
    //! Not part of a shape. -1 is not a valid index the shape array.
    enum { NONE = -1 };
    
    Vertex() : WPoint() {}

    Vertex(coord x, coord y, int shapeno_ = NONE)
    : WPoint(x,y), shapeno(shapeno_) { }
    
    Vertex(WPoint const & wpoint, int shapeno_ = NONE)
    : WPoint(wpoint), shapeno(shapeno_) { }
    
  };
  
  struct Shape
  {
    int startidx;
    int vertices;
    
    Shape(int startidx_ = 0, int vertices_ = 0) 
    : startidx(startidx_), vertices(vertices_) { } 
  };
  
  
  //! array of shapes
  vector<Shape> shapes;
  
  //! array of shape vertices
  vector<Vertex> vertices;

  vector<WPoint> startarea;
  vector<WPoint> goalarea;

  //! Grown vertex
  struct GVertex : public Vertex
  {
    //! array index of nearest actual vertex
    int vertexno;
    GVertex() : Vertex() {}

    GVertex(WPoint pnt, int vertexno_ = NONE)
    : Vertex(pnt), vertexno(vertexno_) {}

    GVertex(Vertex vertex, int vertexno_ = NONE)
    : Vertex(vertex), vertexno(vertexno_) {}

    GVertex(WPoint const & wpoint, int shapeno, int vertexno_)
    : Vertex(wpoint,shapeno), vertexno(vertexno_) {}
  };
  
  //! array of grown shapes
  vector<Shape> gshapes;
  
  //! array of grown vertices
  vector<GVertex> gvertices;
  
  GVertex start;
  GVertex goal;  

protected:

  enum { START = -1, GOAL = -2 };

  //! START or GOAL values or indices of points in the gvertices array that are not inside other polygons
  vector<int> nodes;

  //! visibility graph as an adjacency matrix, indices are the same as "nodes" indices.
  SMatrix<bool> isvisible;

  //! visibility graph as an adjacency matrix, indices are the same as "nodes" indices.
  SMatrix<double> distanceCache;

public:

  //! optimal path of grown vertices, comprised indices into the gvertices array
  vector<int> path;
  
protected:

  static WPoint robot[];


  bool noIntersect
  (
    vector<Shape> & sbefore, vector<GVertex> & vbefore,
    vector<Shape> & safter, vector<GVertex> & vafter
  );
  
  GVertex & get_node(int i);
 
public:

  //! Read obstacle file
  template<typename PointType>
  void readFile(FILE * fp, vector<PointType> & vertices, vector<Shape> * shapes = NULL);

  void describe();
  
  template<class InputIterator>
  void outputShapes(FILE * fp, InputIterator istart, InputIterator iend);

  void outputTargets(FILE * fp);
  void outputVisibility(FILE * fp);
  void outputPath(FILE * fp);
  
};


/////////////////////////////////////////////////////////////////// DEFINITIONS

#include <string>
#include <iostream>

using std::string;
using std::cerr;
using std::endl;


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

template<class InputIterator>
void World::outputShapes(FILE * fp, InputIterator istart, InputIterator iend)
{
  if (istart == iend) return;
  
  int lastshape = istart->shapeno;
  InputIterator firsti = istart;
  
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

#endif
