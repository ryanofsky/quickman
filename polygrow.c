#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "qman.h"
#include <errno.h>
extern int errno;

#define EPSILON 0.0001 /* like zero */

Vertex * obstacles[MAX_POLY];
Vertex * grown[MAX_POLY];
Point starts[MAX_POINTS];
Point goals[MAX_POINTS];

int ct=0; /* global count of actually created obstacles */
int start_ct=0;
int goal_ct=0;


/* assumes decent doubly-linked list as input */
Vertex * free_polygon(Vertex *p) {
  Vertex *q=NULL;
  if (!p) {
    printf("Null pointer passed to free_polygon.  Fine!\n");
    return NULL;
  }
  q=p->next;
  /* 20mg of paranoid */
  if (q==NULL) {
    printf("There is something fundamentally wrong with that vertex list.\n");
    return NULL;
  }
  if (p==q) {
    printf("Self-referential head vertex.\n");
    return NULL;
  }
  while (q!=p) {
    /* hello world.    printf("%p\t%p\t%p\n", p, q->next, q); */
    free(q->prev);
    q=q->next;
  }
  return NULL;
}

/* let C = A - B */
int subtract_vector(Point A, Point B, Point *C) {
  if (!C)
    return -1;
  C->x = A.x - B.x;
  C->y = A.y - B.y;
  return 0;
}

/* let C = A - B ... you know what we mean. */
int subtract_vertex(Vertex *A, Vertex *B, Point *C) {
  if ((!A) || (!B) || (!C))
    return -1;
  return subtract_vector(A->pt, B->pt, C);
}


/* let C = A + B */
int add_vector(Point A, Point B, Point *C) {
  if (!C)
    return -1;
  C->x = A.x + B.x;
  C->y = A.y + B.y;
  return 0;
}

/* let C = A + B ... you know what we mean. */
int add_vertex(Vertex *A, Vertex *B, Point *C) {
  if ((!A) || (!B) || (!C))
    return -1;
  return add_vector(A->pt, B->pt, C);
}

/* A becomes B rotated 90 deg clockwise */
int rot_by_90(Point *A, Point *B) {
  if ((!A) || (!B))
    return -1;
  A->x = B->y;
  A->y = -B->x;
  return 0;
}


double squared_norm(Point A) {
  return A.y * A.y + A.x * A.x;
}

double norm(Point A) {
  return sqrt(squared_norm(A));
}

double distance_squared(Point p1, Point p2) {
  return 
    (p2.y - p1.y)*(p2.y - p1.y) + (p2.x - p1.x)*(p2.x - p1.x);
}

/* given a polygon, based on the angle
   (h->prev, h, h->next)
   remove vertices for which the angle has measure 180 degrees
   reverse the order of vertices if wrong orientation.
   return the new head, in case we removed the original.
   exit with error message if not-a-polygon or concave
*/
Vertex * orient(Vertex *h) {
  /* will exit with error if less than 3 vertices */
  Point u, v;
  Vertex *t; /* as in traversor */
  Vertex *s; /* as in spare */
  int weird=0; /* oriented weirdly: reverse later */
  int nonweird=0; /* oriented normally */
  int justmovedhead; /* essential for loop control */
  double z;
  if (!h) {
    printf("Huge error: head vertex==NULL\n");
    exit(1);
  }
  t=h;
  do {
    justmovedhead=0;
    if ((subtract_vertex(t->next, t, &u)) ||
	(subtract_vertex(t->prev, t, &v)) ||
	((u.x == v.x) && (u.y == v.y))) {
      printf("Polygon has less than 3 real edges.  \
Set to NULL.\n");
      t=free_polygon(t);
      return NULL;
    }
    /* check z component of (u x v) to see orientation.
       nonnegative is what we want. */
    z = u.x * v.y - v.x * u.y;
    /* printf("z is %g\n", z); */
    if ((z<EPSILON) && (z>-EPSILON)) {
      /* take out this vertex.  t will take
       * the value of the next vertex. */
      if (h==t) {
	/* we don't want to lose our head, do we? */
	h=t->next;
	justmovedhead=1;
      }
      printf("Taking out vertex (%g, %g)\n", t->pt.x, t->pt.y);
      t=t->next;
      /* hack.  just so we do not have to use a new pointer. */
      t->prev=t->prev->prev;
      free(t->prev->next);
      t->prev->next=t;
    } else {
      if (z>0) {
	nonweird++;
      } else /* if (z<0) */ {
	weird++;
      }
      t=t->next;
    }
  } while ((t!=h) || justmovedhead);
  
  printf("weird==%d, nonweird==%d\t", weird, nonweird);
  
  if (weird && nonweird) {
    printf("Polygon is concave.  Set to NULL.\n");
    t=free_polygon(t);
    return NULL;
  }
  printf("Verified convexity.");

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
    do {
      /* traverse backwards for fun */
      s=t->prev;
      t->prev=t->next;
      t->next=s;
      t=s;
    } while (t!=h);
    printf("  Reoriented the polygon.");
  }
  printf("\n");
  return h;
}


void read_start(char *startfilename) {
  double x, y;
  char buffer[128];
  char *err;
  char *bufp= &(buffer[0]);
  FILE *fp=fopen(startfilename, "r");
  if (fp==NULL) {
    printf("Could not open %s for reading\n", startfilename);
    exit(1);
  }

  while( (err=fgets(bufp, 128, fp)) ) {
    if (buffer[0] == '#')
      continue; /* skip comments.  pretty cool, heh? */
    if ((*err)=='\n')
      break;
    if (start_ct==MAX_POINTS) {
      printf("Read too many start points. Bye.\n");
      exit(1);
    }
    sscanf(bufp, "%lg %lg", &x, &y);
    starts[start_ct].x = x;
    starts[start_ct].y = y;
    start_ct++;
  }
  fclose(fp);
}

void read_goal(char *goalfilename) {
  double x, y;
  char buffer[128];
  char *bufp= &(buffer[0]);
  FILE *fp=fopen(goalfilename, "r");
  char *err;
  if (fp==NULL) {
    printf("Could not open %s for reading\n", goalfilename);
    exit(1);
  }

  while( (err=fgets(bufp, 128, fp)) ) {
    if (buffer[0] == '#')
      continue; /* skip comments */
    if ((*err)=='\n')
      break;
    if (goal_ct==MAX_POINTS) {
      printf("Read too many goal points. Bye.\n");
      exit(1);
    }
    sscanf(bufp, "%lg %lg", &x, &y);
    goals[start_ct].x = x;
    goals[start_ct].y = y;
    goal_ct++;
  }
  fclose(fp);
}


/* read obstacles from file. */
void read_obstacles(char *obsfilename) {
  int flag=0; /* ugly hack */
  double x, y;
  FILE *fp;
  char buffer[128];
  char *bufp= &(buffer[0]);
  char *err;
  Vertex *h=NULL;
  Vertex *p;
  Vertex **q= &(obstacles[0]);
  fp = fopen(obsfilename, "r");
  if (fp==NULL) {
    printf("Could not open %s for reading\n", obsfilename);
    exit(1);
  }
  while( (err = fgets(bufp, 128, fp)) ) {
    if (buffer[0] == '#') {
      continue; /* skip comments, but not whitespace */
    }
    if ((*err == '\n') || (buffer[0] == '\n') || (buffer[0]== ' ')
	|| feof(fp)) {
      if (!flag) {
	/*
	 * printf("read newline == we are done with a polygon.\n");
	 */

	/* we do not need the last vertex to be the same! */
	/* so we get rid of current. */
	ct++;
	if (ct==MAX_POLY) {
	  printf("Read too many polygons.  Bye.\n");
	  exit(1);
	}
	/* reminder: we are at a new node with no coords read */
	p=p->prev;
	free(p->next);
	/* in the test files, it is the case that the
	   first vertex is the same as the last.  
	   wrong, wrong, wrong... */
	if ((p->pt.x == h->pt.x) &&
	    (p->pt.y == h->pt.y)) {
	  p=p->prev;
	  free(p->next);
	}
	p->next=h;
	h->prev=p;
	// *q=h;
	*q = orient(h); /* two birds in one line, kind of */
	q= &(obstacles[ct]); /* a bit inefficient, but safe! */
	h=NULL;
      }
      flag = 1;
    } else {
      sscanf(bufp, "%lg %lg", &x, &y);
      /* printf("x is %g, y is %g\n", x, y); */
      if (h==NULL) {
	h=(Vertex *)malloc(sizeof(Vertex));
	p=h;
      } 
      p->pt.x = x;
      p->pt.y = y;
      p->next = (Vertex *)malloc(sizeof(Vertex));
      p->next->prev = p;
      p = p->next;
      flag = 0;
    }
  }

  /*  printf("Closing obstacles file\n"); */
  fclose(fp);
  /*  printf("Read %d polygons.\n", ct); */
}


/* normalize so that norm of vector *A becomes d */
int diametrize(Point *A, double d) {
  double scale, nrm;
  if (!A)
    return -1;
  if ((nrm = norm(*A)) == 0.0)
    return 0; 
  /* to avoid div by 0. *A has the right value too. */
  scale = d/nrm;
  (A->x) *= scale;
  (A->y) *= scale;
  return 0;
}


/* returns grown version of h, keeping h intact */
/* pre: h is a convex polygon in counterclockwise order */
Vertex * polygrow(Vertex * h, double amount) {
  /* error checking not fully implemented */
  /*  int done; */
  double j, k;
  Vertex *g=NULL;
  Vertex *p;
  Vertex *q;
  Point mCA, mAB, mCE, mBD; /* treated like vectors */
  Point D, E; /* treated like points */
  if ((!h) || (amount<=0))
    return g;
  p=h;
  g=(Vertex *)malloc(sizeof(Vertex));
  q=g;
  /* foreach vertex in polygon */
  do {
    /* calculate (p->pt)' */
    if (!(p->prev)) {
      printf("no prev!\n");
      exit(1);
    }
    if (!(p->next)) {
      printf("no next!\n");
      exit(1);
    }

    if (subtract_vertex(p, p->prev, &mCA))
      printf("error\n");
    if (subtract_vertex(p->next, p, &mAB))
      printf("error\n");
    rot_by_90(&mCE, &mCA);
    rot_by_90(&mBD, &mAB);
    diametrize(&mCE, amount);
    diametrize(&mBD, amount);
    add_vector(p->prev->pt, mCE, &E);
    add_vector(p->next->pt, mBD, &D);
    
    /* NOT CHECKING FOR pathological (i.e., non-convex) cases */
    /* this means that we assume that the lines intersect. */

    /* the point we want (A') is the intersection of lines */
    
    /* (1) <Dx, Dy> + k <mABx, mABy>
     *  and
     * (2) <Ex, Ey> + j <mCAx, mCAy>
    
     * for the point of intersection:
     * x-coord = Dx + k (mABx) = Ex + j (mCAx)
     * y-coord = Dy + k (mABy) = Ey + j (mCAy)
     * for unique (k, j)
     */
    
    j= (D.x * mAB.y - D.y * mAB.x - E.x * mAB.y + E.y * mAB.x) /
      (mCA.x * mAB.y - mCA.y * mAB.x);
    
    if(mAB.y != 0.0) {
      k = (E.y + j * mCA.y - D.y)/mAB.y;
    } else if(mAB.x != 0.0) {
      k = (E.x + j * mCA.x - D.x)/mAB.x;
    } else {
      /* oh well.  so much for not checking. */
      printf("big bug: two adjacent vertices coincide! (%g, %g)\n",
	     p->pt.x, p->pt.y);
      exit(1);
    }
    
    q->pt.x = D.x + k * mAB.x;
    q->pt.y = D.y + k * mAB.y;
    /* done calculating q->pt.x and q->pt.y */
    
    if (!(p->next)) {
      printf("error\n");
      break;
    }
    
    if ((p->next) == h) {
      q->next=g;
      g->prev=q;
    } else {
      q->next=(Vertex *)malloc(sizeof(Vertex));
      q->next->prev=q;
      q=q->next;
    }

    p=p->next;
  } while (p!=h);
  
  return g;
};


void print_poly(Vertex *h) {
  Vertex *p=h;
  if (!h) {
    printf("NULL pointer passed to print_poly.\n\n");
    return;
  }
  do {
    printf("%g\t%g\n", p->pt.x, p->pt.y);
    p=p->next;
  } while (p!=h);
  printf("\n");
}


Vertex * points_to_poly (Point vertices[], int n) {
  /* given array of Point and the size to read from
     that array, generates a polygon represented as
     a doubly-linked circular list */
  int i;
  Vertex *p;
  Vertex *h;
  if (n<3)
    return NULL; /* not a polygon */
  h=(Vertex *)malloc(sizeof(Vertex));
  p=h;
  for (i=0; i<n-1; i++) {
    p->pt.x = vertices[i].x;
    p->pt.y = vertices[i].y;
    p->next=(Vertex *)malloc(sizeof(Vertex));
    p->next->prev=p;
    p=p->next;
  }
  /* hack for bigger, faster code */
  p->pt.x = vertices[i].x;
  p->pt.y = vertices[i].y;
  p->next=h;
  h->prev=p;
  return h;
}


int main(int argc, char *argv[]) {
  double diameter;
  int i;
  
  if (argc>2) {
    printf("Usage:\n\
%s [optional: <robot_diameter>]\n", argv[0]);
    exit(1);
  }

  if (argc==2) {
    diameter = atof(argv[1]);
    if (errno || (diameter<=0)) {
      printf("<robot_diameter> must be a positive integer!\n");
      exit(1);
    }
  } else diameter = DEFAULT_DIAMETER;

  //  read_obstacles("obstacles.txt");
  read_obstacles("bad_obstacles.txt");
  for(i=0; i<ct; i++) {
    printf("Input %d was:\n", i);
    print_poly(obstacles[i]);
    
    grown[i]=polygrow(obstacles[i], diameter);
  }

  for(i=0; i<ct; i++) {
    printf("Obstacle %d now is:\n", i);
    print_poly(grown[i]);
  }

  read_start("start.txt");
  printf("Read %d start points.\n", start_ct);
  read_goal("goal.txt");
  printf("Read %d goal points.\n", goal_ct);
  return 0;

}












