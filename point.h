#ifndef point_h
#define point_h

#include <vector>

using std::vector;

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
  bool inline equals(Point<T> const & c) const;
};

//! Line segments connecting P1 Q1 and P2 Q2 intersect
template<typename T>
bool inline linesIntersect(Point<T> P1, Point<T> Q1, Point<T> P2, Point<T> Q2);  

//! Find the convex hull of a set of points. Could be more generic...
template<typename T>
void convexHull(vector<Point<T> > const & points, vector<Point<T> > & hull);

//! Helper macro
#define DIM(x) (sizeof(x)/sizeof(x[0]))

#endif

