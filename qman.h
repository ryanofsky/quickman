#ifndef qman_h
#define qman_h

#include "point.h"

/* qman.h */
/* data structures and stuff */

#define MAX_POLY 200
/* maximum number of polygons that our program wants to consider */
#define MAX_POINTS 200
/* only for the start and end ones, every polygon can be of arbitrary size */

#define DEFAULT_DIAMETER 662.87253676706202790366913417296
/* calculated from "370 x 550  mm." */


typedef struct il {
  int v;
  struct il *next;
} IntList;

typedef struct vd {
  int v;
  double d;
} IntDist;

typedef struct el {
  int v;
  double d;
  struct el *next;
} EdgeList;

typedef struct vert {
  Point<double> pt;
  struct vert *next;
  struct vert *prev;
} Vertex;
/* this representation is very handy for polygon growing */
/* each polygon is represented as a doubly-linked circular
 * list of Vertices */

#endif