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
  
};

#endif
