#include "point.h"

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

/*
template<typename T>
bool Point<T>::equals(Point<T> c) const
{
  return x == c.x && y == c.y;
}
*/

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
  if(P.y == Q.y) /* horizontal */
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

template<typename T>
void convexHull(vector<Point<T> > const & points, vector<Point<T> > & hull)
{
  // hasty but straightforward implementation of graham's convex hull
  // algorithm given in the notes.
  
  int l = points.length;
  
  // find rightmost, lowest point
  int pivot = 0;
  for(int i = 1; i < l; ++i)
  {
    if (point[i].x > point[pivot].x || (point[i].x == point[pivot].x && point[i].y > point[pivot].y))
      pivot = i;
  }
  
  // find angles about pivot, store them in a vector for quick comparison
  
  class cpoint // cached point
  {
    Point<T> point;
    double angle;
    cpoint(Point<T> point_, double angle_) : point(point_), angle(angle_) { } 
  }
  
  typedef vector<cpoint> vcpoint;
  vcpoint cpoints(l);

  for(int i = 0; i < l; ++i)
  {
    if (point[i].equals(pivot))
      cpoints[i] = 0;
    else
      cpoints[i] = cpoint(point[i], atan2(point[i].y - pivot.y, point[i].x - pivot[x]));
  }

  class lessthan // comparison functor
  {
  public:
    lessthan(Point<T> pivot_) : pivot(pivot_) { } 
    Point<T> pivot;
    
    operator(cpoint a, cpoint b)
    {
      if (a.angle != b.angle)
        return a.angle < b.angle;
      else
        return pivot.distanceTo(a.c) < pivot.distanceTo(b.c);
    }
  }
  
  sort(cpoints.begin(), cpoints.end(), lessthan(points[pivot]));
  
  hull.resize(0);
  hull.push_back(cpoints[0].point);
  hull.push_back(points.back()->c);
  
  int i = 1;
  while(i < N)
  {
    // top two elements: top and ptop
    vcpoint::const_iterator top(points.end());
    --top;
    vcpoint::const_iterator ptop(top);
    --ptop;
    
    vector<cpoint>
    if (cpoints[i].point.line_side(*ptop, *top) & Point<T>::LEFT_SIDE)
    {
      hull.push_back(cpoints[i].point);
      ++i;
    }
    else
      hull.pop_back();
  }
}
