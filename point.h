#ifndef point_h
#define point_h

#include "general.h"
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
  
  Point(T x_ = 0, T y_ = 0)
  : x(x_), y(y_) {}

  //! Distance to another point
  double inline distanceTo(Point c) const;
  
  // Shortest distance between this point and line segment PQ
  double inline distanceTo(Point P, Point Q) const;
  
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
  
  //! Convenient scalar multiplication function
  Point<T> inline operator*(T c) const; 
  
  //! Convenient scalar division function
  Point<T> inline operator/(T c) const; 
  
  //! Point has the same coordinates as c
  bool inline equals(Point<T> c) const;
  
  /* A becomes B rotated 90 deg clockwise */
  Point<T> rot_by_90() const;
  
  /* normalize so that norm of vector *this becomes d */
  bool diametrize(double d)
  {
    double scale, nrm;
    if ((nrm = norm()) == 0.0)
      return false;
    /* to avoid div by 0. *A has the right value too. */
    scale = d/nrm;
    x *= scale;
    y *= scale;
    return 0;
  }
  
  double squared_norm()
  {
    return y * y + x * x;
  }

  double norm()
  {
    return sqrt(squared_norm());
  }

  // copy constructor  
  template<typename U>
  Point(Point<U> const & p)
  {
    x = (T)p.x;
    y = (T)p.y;
  }

  // assignment constructor
  template<typename U>
  Point<T> & operator=(Point<U> const & p)
  {
    x = (T)p.x;
    y = (T)p.y;
    return *this;
  }
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
Point<T> Point<T>::operator/(T c) const
{
  return Point(x/c, y/c);
}

template<typename T>
Point<T> Point<T>::rot_by_90() const
{
  return Point(y,-x);
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
double Point<T>::distanceTo(Point<T> P, Point<T> Q) const
{
  // Point on the line segment P, Q are described by this parametric eq
  // L = P + u (Q - P)
  // where P, L are points, u is a scalar in [0,1]
  //
  // the closest point on L is where the line from L to *this is perpendicular
  // (this - L) dot (Q  - P) = 0
  //
  // solve for u with these two equations
  // 
  // (this - P - u (Q - P)) dot (Q - P) = 0
  //
  // (this.x - P.x - u(Q.x - P.x))(Q.x - P.x) + (this.y - P.y - u(Q.y - P.y))(Q.y - P.y) = 0
  //
  // (this.x - P.x)(Q.x - P.x) + (this.y - P.y)(Q.y - P.y) - u ((Q.x - P.x)^2 + (Q.y - P.y)^2)
  //
  // u = ((this.x - P.x)(Q.x - P.x) + (this.y - P.y)(Q.y - P.y)) / ((Q.x - P.x)^2 + (Q.y - P.y)^2)
  // 
  
  if (P.equals(Q)) return distanceTo(P);
  
  double dx = Q.x - P.x;
  double dy = Q.y - P.y;
  double u = ((double)(this->x - P.x) * dx + (double)(this->y - P.y) * dy) / (dx*dx + dy*dy);

  if (u < 0.0)
    u = 0;
  else if (u > 1.0)
    u = 1.0;
  
  dx = (double)P.x + u * (double)(Q.x - P.x) - (double)this->x;
  dy = (double)P.y + u * (double)(Q.y - P.y) - (double)this->y;
  return sqrt(dx*dx + dy*dy);
}

template<typename T>
int Point<T>::line_side(Point<T> P, Point<T> Q) const
{
  if(P.y == Q.y) // horizontal
    if(this->y < P.y)
      return RIGHT_SIDE;
    else if(this->y > P.y)
      return LEFT_SIDE;
    else
      return (this->x == P.x ) ? LEFT_SIDE | RIGHT_SIDE : 0;
  
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

/* this is sedgewick's nicer ccw algorithm, 
  but it doesn't seem to work for certain cases
  like P.x == Q.x == this->x
 
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
 */

  int r = line_side(P,Q);
  if (P.y > Q.y || (P.y == Q.y && P.x > Q.x))
  {
    if (r == LEFT_SIDE)
      return RIGHT_SIDE;
    else if (r == RIGHT_SIDE)
      return LEFT_SIDE;
  }
  return r;
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


 // outside because of G++
template<class InputIterator>
class _convexHull_cpointLessThan
{
  typedef _convexHull_cpoint<InputIterator> cpoint;
public:

  bool operator()(cpoint a, cpoint b)
  {
    if (a.angle != b.angle)
      return a.angle < b.angle;
    else
    {
      bool acloser = a.point->distanceTo(*pivot) < b.point->distanceTo(*pivot);
      return (a.angle <= PI / 2.0) ? acloser : !acloser;
    }  
  }
  
  _convexHull_cpointLessThan(InputIterator pivot_)
  : pivot(pivot_) { } 
  
  InputIterator pivot;
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
  typedef _convexHull_cpointLessThan<InputIterator> cpointLessThan;
  
  assert(pointEnd - pointStart >= 3); // need at least 3 points
  
  // pivot will be the lowest, rightmost point
  InputIterator pivot = pointStart;
  for(InputIterator i = pointStart + 1; i != pointEnd; ++i)
  {
    if (i->y < pivot->y || (i->y == pivot->y && i->x > pivot->x))
      pivot = i;
  }
  
  vector<cpoint> cpoints; // not really efficient if the function is called repeatedly
  typedef vector<cpoint>::const_iterator icpoint;
  
  cpoints.push_back(cpoint(pivot,0));
  
  for(InputIterator i = pointStart; i != pointEnd; ++i)
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

  //for (icpoint p = cpoints.begin(); p !=  cpoints.end(); ++p)
  //  cerr << (p - cpoints.begin())<< '\t' << p->point->x << '\t' << p->point->y << '\t' << p->angle<< endl;
 
  OutputIterator hull = hullStart;
  int i = 0;
  int hlkjh = cpoints.end() - cpoints.begin();  
  while(i < 2) // add first two points
  {
    *hull = *cpoints[i].point;
    ++hull; ++i;
  }

  typedef vector<cpoint>::const_iterator icpoints;
  while(i < cpoints.size())
  {
    
    OutputIterator _hp = hull - 1;
    OutputIterator _hpp = hull - hullStart > 1 ? hull -2 : hull - 1;
    
    
    assert(hull - hullStart > 0); // hull must have at least one point
    if (hull - hullStart ==  1 || cpoints[i].point->line_rside(*(hull-2), *(hull-1))  == Point<int>::LEFT_SIDE)
    {

      assert(hull >= hullStart && hull - hullStart < cpoints.size());
      *hull = *cpoints[i].point;
      ++hull; ++i;
    }
    else
      --hull;
  };
  
  assert(hull - hullStart >= 2);
  
  if (hull - hullStart > 2 &&  hullStart->line_rside(*(hull-2), *(hull-1))  != Point<int>::LEFT_SIDE)
    --hull;

  return hull;
}

/* for each polygon vertex, based on the angle of
   previous and next vertices
   remove vertices for which the angle has measure 180 degrees
   reverse the order of vertices if wrong orientation.
   return the new head, in case we removed the original.
   exit with error message if not-a-polygon or concave
*/
template<class PointType, class RandomAccessIterator>
bool orient_poly(RandomAccessIterator pointStart, RandomAccessIterator pointEnd)
{
  if (pointEnd - pointStart < 2) return true;

  const double EPSILON = 0.0001;

  PointType u, v;
  RandomAccessIterator t; /* as in traversor */
  //Vertex *s; /* as in spare */
  int weird=0; /* oriented weirdly: reverse later */
  int nonweird=0; /* oriented normally */
  int justmovedhead; /* essential for loop control */
  double z;

  for(t = pointStart; t != pointEnd; ++t)
  {
    RandomAccessIterator prev = t == pointStart ? pointEnd - 1 : t - 1;
    RandomAccessIterator next = t + 1 == pointEnd ? pointStart : t + 1;

    justmovedhead=0;
    
    u = *next - *t;
    v = *prev - *t;
    
    /* check z component of (u x v) to see orientation.
       nonnegative is what we want. */
    z = u.x * v.y - v.x * u.y;
    /* printf("z is %g\n", z); */
    
     if ((z<EPSILON) && (z>-EPSILON)) {
      // RWH: skip for array implmemtation
      //
      // /* take out this vertex.  t will take
      //  * the value of the next vertex. */
      // if (h==t) {
            // /* we don't want to lose our head, do we? */
            // h=t->next;
            // justmovedhead=1;
      // }
      // printf("Taking out vertex (%g, %g)\n", t->pt.x, t->pt.y);
      // t=t->next;
      // /* hack.  just so we do not have to use a new pointer. */
      // t->prev=t->prev->prev;
      // free(t->prev->next);
      // t->prev->next=t;
    } else {
      if (z>0) {
        nonweird++;
      } else /* if (z<0) */ {
        weird++;
      }
    }
  }
  
  //printf("weird==%d, nonweird==%d\t", weird, nonweird);
  
  if (weird && nonweird) {
    return false;
    //printf("Polygon is concave.  Set to NULL.\n");
  }
  // printf("Verified convexity.");

  if (weird) {
    /* Need to reverse the order of the polygon */
    /* ...because Deniz could not figure out in finite time
       a good way to both check and fix orientation in one pass.  
       the greatest caveat was the act of removing a vertex,
       what if the vertex to be removed is the one before
       the last, etc.  Also, it is probably more efficient 
       this way since by default, we are not expecting
       polygons to be weird. */
    /* just swap the prev and next pointers. */
    reverse(pointStart,pointEnd);
    //printf("  Reoriented the polygon.");
  }
  return true;
}

#endif
