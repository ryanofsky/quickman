#ifndef point_h
#define point_h

#include "math.h"
#include <vector>
#include <algorithm>
#include <assert.h>

#define PI 3.14159265358979323846264338328

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
  typedef T CoordType;
  
  T x,y;
  
  Point(int x_ = 0, int y_ = 0)
  : x(x_), y(y_) {}

  //! Distance to another point
  double inline distanceTo(Point c) const;
  
  //! Orientation flags
  enum Orientation { LEFT_SIDE = 0x1, RIGHT_SIDE = 0x2 }; 
  
  //! Returns this point's orientation relative to a line through PQ
  int inline line_side(Point P, Point Q) const;

  //! You are on a road walking from P to Q... Is this point to the left or to the right of the road?
  int inline line_rside(Point P, Point Q) const;
  
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
int Point<T>::line_rside(Point<T> P, Point<T> Q) const
{
  T dx1 = Q.x-P.x;
  T dy1 = Q.y-P.y; 
  T dx2 = this->x - P.x;
  T dy2 = this->y - P.y; 
  T x1y2 = dx1 * dy2;
  T x2y1 = dy1 * dx2;
  
  if (x1y2 > x2y1)
    return LEFT_SIDE;
  else if (x1y2 < x2y1)
    return RIGHT_SIDE; 
  else
  {
    if (dx1*dx2<0 || dy1*dy2<0)
      return RIGHT_SIDE;
    else if (dx1*dx1+dy1*dy1 >= dx2*dx2+dy2*dy2)
      return LEFT_SIDE | RIGHT_SIDE;
    else 
      return LEFT_SIDE;
  }
} 

template<typename T>
bool linesIntersect(Point<T> P1, Point<T> Q1, Point<T> P2, Point<T> Q2)
{
  return !(P2.line_rside(P1, Q1) & Q2.line_rside(P1, Q1) || P1.line_rside(P2, Q2) & Q1.line_rside(P2, Q2));
}

// This class is only used inside the convexHull function, but for some reason
// metrowerks won't allow me to instantiate it there if it is declared inside.

template<class T>
struct _convexHull_cpoint
{
  T point;
  double angle;
  
  _convexHull_cpoint() {}
  
  _convexHull_cpoint(T point_, double angle_)
  : point(point_), angle(angle_) { }
};

template<class InputIterator, class OutputIterator>
OutputIterator convexHull(InputIterator pointStart, InputIterator pointEnd, OutputIterator hullStart)
{
  // orientation:
  //
  //    -x <-----> +x
  //
  //       +y ^
  //          |
  //          |
  //       -y v
  
  typedef _convexHull_cpoint<InputIterator> cpoint;
  
  assert(pointEnd - pointStart >= 3); // need at least 3 points
  
  // pivot will be the lowest, rightmost point
  InputIterator pivot = pointStart;
  for(InputIterator i = pointStart + 1; i != pointEnd; ++i)
  {
    if (i->y < pivot->y || (i->y == pivot->y && i->x > pivot->x))
      pivot = i;
  }
  
  class cpointLessThan
  {
  public:
    
    operator()(cpoint a, cpoint b)
    {
      if (a.angle != b.angle)
        return a.angle < b.angle;
      else
        return a.point->distanceTo(*pivot) < a.point->distanceTo(*pivot);
    }
    
    cpointLessThan(InputIterator pivot_)
    : pivot(pivot_) { } 
    
    InputIterator pivot;
  };

  vector<cpoint> cpoints; // not really efficient if the function is called repeatedly
  typedef vector<cpoint>::iterator icpoint;
  
  cpoints.push_back(cpoint(pivot,0));
  
  for(InputIterator i = pointStart + 1; i != pointEnd; ++i)
  {
    if (!i->equals(*pivot)) // can't find the angle between a point and itself
    {
      double a = atan2(i->y - pivot->y, i->x - pivot->x);
      // sanity check, angle from the pivot must be between 0 and PI since pivot is the lowest point
      assert(a >= 0 && a <= PI); 
      cpoints.push_back(cpoint(i,a));
    }  
  };

  sort(cpoints.begin() + 1, cpoints.end(), cpointLessThan(pivot));
  
  OutputIterator hull = hullStart;
  int i = 0;
  
  while(i < 2) // add first two points
  {
    *hull = *cpoints[i].point;
    ++hull; ++i;
  }

  typedef vector<cpoint>::iterator icpoints;
  while(i < cpoints.size())
  {
    OutputIterator top = hull - 1;
    OutputIterator ptop = hull - 2;
    
    if(cpoints[i].point->line_rside(*ptop, *top) == Point<int>::LEFT_SIDE)
    {
      *hull = *cpoints[i].point;
      ++hull; ++i;
    }
    else
      --hull;
  };
  return hull;
}

#endif

