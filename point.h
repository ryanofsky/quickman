#ifndef point_h
#define point_h

#include "math.h"
#include <vector>
#include <algorithm>

using std::sort;
using std::vector;

////////////////////////////////////////////////////////////////// DECLARATIONS

//! Helper macro
#define DIM(x) (sizeof(x)/sizeof(x[0]))

/*! Point class

   This class is parameterized so we can easily test out the speed
   of different data types. 
   
   For example, it might be significantly faster to test for line
   intersection using integer math instead of floating point math.
*/

template<typename T>
struct Point
{
  T x,y;
  
  Point(int x_ = 0, int y_ = 0)
  : x(x_), y(y_) {}

  //! Distance to another point
  double inline distanceTo(Point c) const;
  
  //! Orientation flags
  enum Orientation { LEFT_SIDE = 0x1, RIGHT_SIDE = 0x2 }; 
  
  //! Returns this point's orientation relative to a line through PQ
  int inline line_side(Point P, Point Q) const;
  
  //! Convenient addition function
  Point<T> inline operator+(Point<T> c) const;

  //! Convenient subtraction function
  Point<T> inline operator-(Point<T> c) const;   
  
  //! Point has the same coordinates as c
  bool inline equals(Point<T> c) const;
};

//! Line segments connecting P1 Q1 and P2 Q2 intersect
template<typename T>
bool inline linesIntersect(Point<T> P1, Point<T> Q1, Point<T> P2, Point<T> Q2);  

//! Find the convex hull of a set of points. Could be more generic...
template<typename T>
void convexHull(vector<Point<T> > const & points, vector<Point<T> > & hull);

/////////////////////////////////////////////////////////////////// DEFINITIONS


template<typename T>
Point<T> Point<T>::operator+(Point<T> c) const
{
  return Point(x + c.x, y + c.y);
}

template<typename T>
Point<T> Point<T>::operator-(Point<T> c) const
{
  return Point(x - c.x, y - c.y);
}

template<typename T>
bool Point<T>::equals(Point<T> c) const
{
  return x == c.x && y == c.y;
}

template<typename T>
double Point<T>::distanceTo(Point c) const
{
  // currently always uses double,
  // could be parameterized if we find a fast
  // integer square root function.
  double dx = x - c.x;
  double dy = y - c.y;
  return sqrt(dx*dx + dy*dy);
}

template<typename T>
int Point<T>::line_side(Point<T> P, Point<T> Q) const
{
  if(P.y == Q.y) // horizontal
    if(this->y < P.y)
      return LEFT_SIDE;
    else if(this->y > P.y)
      return RIGHT_SIDE;
    else
      return LEFT_SIDE | RIGHT_SIDE;
  
  // x coordinate of the point on the line where y = this->y
  T x = P.x == Q.x ? P.x : P.x + (Q.x-P.x) * (this->y-P.y) / (Q.y-P.y);
  if(this->x < x)
    return LEFT_SIDE;
  else if(this->x > x)
    return RIGHT_SIDE;
  else
    return LEFT_SIDE | RIGHT_SIDE;
}

template<typename T>
bool linesIntersect(Point<T> P1, Point<T> Q1, Point<T> P2, Point<T> Q2)
{
  return !(P2.line_side(P1, Q1) & Q2.line_side(P1, Q1) || P1.line_side(P2, Q2) & Q1.line_side(P2, Q2));
}

// This class is only used inside the convexHull function, but for some reason
// metrowerks won't allow me to instantiate it there if it is declared inside.

template<typename T>
struct _convexHull_cpoint // cached point
{
  typedef Point<T> TPoint;
  TPoint point;
  double angle;
  _convexHull_cpoint() { }
  _convexHull_cpoint(TPoint point_, double angle_) : point(point_), angle(angle_) { } 
};

template<typename T>
void convexHull(vector<Point<T> > const & points, vector<Point<T> > & hull)
{
  typedef Point<T> TPoint;
  typedef _convexHull_cpoint<T> cpoint; // cached point
  
  // hasty but straightforward implementation of graham's convex hull
  // algorithm given in the notes.
  
  int l = points.size();
  
  // find index of rightmost, lowest point
  int pivoti = 0;
  for(int i = 1; i < l; ++i)
  {
    if (points[i].x > points[pivoti].x || (points[i].x == points[pivoti].x && points[i].y > points[pivoti].y))
      pivoti = i;
  }
  TPoint pivot = points[pivoti];
  

  // find angles about pivot, store them in a vector for quick comparison
  typedef vector<cpoint> vcpoint;
  vcpoint cpoints(l);

  // find angles between points
  for(int i = 0; i < l; ++i)
  {
    if (pivot.equals(points[i])) // special case, no angle between overlapping points
      cpoints[i] = cpoint(points[i],0);
    else
    {
      cpoints[i] = cpoint(points[i],atan2(points[i].y - pivot.y, points[i].x - pivot.x));
    }  
  };
  
  class lessthan // comparison functor
  {
  public:
    lessthan(Point<T> pivot_) : pivot(pivot_) { } 
    Point<T> pivot;
    
    operator()(cpoint a, cpoint b)
    {
      if (a.angle != b.angle)
        return a.angle < b.angle;
      else
        return pivot.distanceTo(a.point) < pivot.distanceTo(b.point);
    }
  };
  
  sort(cpoints.begin(), cpoints.end(), lessthan(pivot));
  
  hull.resize(0);
  hull.push_back(cpoints[0].point);
  hull.push_back(cpoints.back().point);
  
  int i = 1;
  while(i < cpoints.size())
  {
    // top two elements: top and ptop
    vcpoint::const_iterator top(cpoints.end());
    --top;
    vcpoint::const_iterator ptop(top);
    --ptop;
    
    if(cpoints[i].point.line_side(ptop->point, top->point) & TPoint::LEFT_SIDE)
    {
      hull.push_back(cpoints[i].point);
      ++i;
    }
    else
      hull.pop_back();
  }
}

#endif

