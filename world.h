#ifndef world_h
#define world_h

#include <stdio.h>
#include "point.h"
#include "smatrix.h"

class World
{
public:

  //! Read obstacle file
  void readFile(FILE * fp);

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
    
    explicit Vertex(WPoint const & wpoint, int shapeno_ = NONE)
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

  //! Grown vertex
  struct GVertex : public Vertex
  {
    //! array index of nearest actual vertex
    int vertexno;
    GVertex() : Vertex() {}

    GVertex(Vertex vertex, int vertexno_)
    : Vertex(vertex), vertexno(vertexno_) {}

    GVertex(WPoint const & wpoint, int shapeno, int vertexno_)
    : Vertex(wpoint,shapeno), vertexno(vertexno_) {}
  };
  
  //! array of grown shapes
  vector<Shape> gshapes;
  
  //! array of grown vertices
  vector<GVertex> gvertices;

protected:

  //! visibility graph as an adjacency matrix
  SMatrix<bool> visibility;

public:

  //! optimal path of grown vertices
  vector<int> path;
  
protected:

  static WPoint robot[];
  
public:

  void describe();
};

#endif
