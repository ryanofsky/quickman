/*

                                     - Dr. Wily  */

#include "saphira.h"
#include "qman.h"

#define DONE_MOVING 20
#define ROTATING    24
#define MOVING      28
#define GOAL        32

extern int ct;
extern Vertex *obstacles[];

Vertex *polygrow(Vertex *old, int diameter);

Point robot_pos;
Point *vertices;
IntList *path = 0;
IntList *current = 0;

int angle_between(Point p1, Point p2) {
  return (int)(sfRadToDeg(atan2(p2.y - p1.y, p2.x - p1.x)));
}

int distance_between(Point p1, Point p2) {
  return (int)sqrt((p2.y - p1.y)*(p2.y - p1.y) + (p2.x - p1.x)*(p2.x - p1.x));
}

void point_print(Point p) {
  printf("(%f, %f)\n", p.x, p.y);
}

IntList *int_insert(IntList *l, int x) {
  IntList *i = malloc(sizeof(IntList));
  i->v = x;
  i->next = l;
  return i;
}

void int_append(IntList *l, int x) {
  if(l->next) int_append(l->next, x);
  else if(l) {
    l->next = malloc(sizeof(IntList));
    l->next->v = x;
    l->next->next = 0;
  }
}

void int_print(IntList *l) {
  if(l) {
    printf("[%d]->", l->v);
    int_print(l->next);
  }
  else printf(">\n");
}

void id_print(IntDist *l, int n) {
  int i;
  for(i=0; i<n; i++) {
    printf("[%d %f] ", l->v, l->d);
  }
  printf("\n");
}

Point get_robot_position() {
  Point pos;
  pos.x = 4235 - sfRobot.ay;
  pos.y = sfRobot.ax + 475;
  return pos;
}

void follow_points(void) {
  switch(process_state) {
  case sfINIT:
    process_state = DONE_MOVING;
    break;
  case DONE_MOVING:
    robot_pos.x = 4235 - sfRobot.ay;
    robot_pos.y = sfRobot.ax + 475;
    sfSetHeading(angle_between(robot_pos, vertices[current->v])- 90);
    process_state = ROTATING;
    break;
  case ROTATING:
    if (sfDoneHeading(10)) {
      robot_pos.x = 4235 - sfRobot.ay;
      robot_pos.y = sfRobot.ax + 475;
      sfSetPosition(distance_between(robot_pos, vertices[current->v]));
      process_state = MOVING;
    }
    break;
  case MOVING:
    if(sfDonePosition(100)) {
      current = current->next;
      if(!current) process_state = GOAL;
      else { /* process_state = DONE_MOVING;*/
	robot_pos.x = 4235 - sfRobot.ay;
	robot_pos.y = sfRobot.ax + 475;
	sfSetHeading(angle_between(robot_pos, vertices[current->v])- 90);
	process_state = ROTATING;	
      }
    }
    break;
  case GOAL:
    break;
  }
}

#define KEY(x) (d[heap[x]].d)
#define SWAP(x,y) (x) ^= (y); (y) ^= (x); (x) ^= (y)
#define INFINITY 100000

void heapify(int *heap, IntDist *d, int heapsize, int i) {
  int l = i*2;
  int r = i*2 + 1;
  int min;
  if(l < heapsize && KEY(l) < KEY(i))
    min = l;
  else
    min = i;
  if(r < heapsize && KEY(r) < KEY(min))
    min = r;
  if(min != i) {
    SWAP(heap[min], heap[i]);
    heapify(heap, d, heapsize, min);
  }
}

int extract_min(int *heap, IntDist *d, int *heapsize) {
  int min = heap[0];
  int i;

  *heapsize = *heapsize-1;
  heap[0] = heap[*heapsize];
  heapify(heap, d, *heapsize, 0);

  return min;
}

void build_heap(int *heap, IntDist *d, int heapsize) {
  int i;
  for(i=heapsize/2; i>0; i--)
    heapify(heap, d, heapsize, i);
}

/* given the visibility graph as an array of edge lists and the start node,
   return the shortest path as an array in which d[i] gives the reverse
   shortest path from start to i */
IntDist *dijkstra(EdgeList **vis, int num, int start) {
  IntDist *d = malloc(num*sizeof(IntDist));
  int heap[num];
  int heapsize = num;
  int i;

  for(i=0; i<num; i++) edge_print(vis[i]);

  for(i=0; i<num; i++) {
    d[i].d = INFINITY;
    d[i].v = -1;
    heap[i] = i;
  }
  d[start].d = 0;
  build_heap(heap, d, heapsize);

  while(heapsize > 0) {
    EdgeList *edge;
    i = extract_min(heap, d, &heapsize);
    edge = vis[i];
    while(edge) {
      if(d[edge->v].d > d[i].d + edge->d) {
	d[edge->v].d = d[i].d + edge->d;
	d[edge->v].v = i;
	heapify(heap, d, heapsize, 0);
      }
      edge = edge->next;
    }
  }

  id_print(d, num);
  
  return d;
}

/* takes the d array output by dijkstra and creates a linked list path */
IntList *d_to_path(IntDist *d, int goal) {
  IntList *path = 0;

  while(goal >= 0) {
    path = int_insert(path, goal);
    goal = d[goal].v;
  }

  return path;
}

EdgeList *edge_insert(EdgeList *e, int v, double d) {
  EdgeList *i = malloc(sizeof(EdgeList));
  i->v = v;
  i->d = d;
  i->next = e;
  return i;
}

void edge_print(EdgeList *e) {
  if(e) {
    printf("[%d|%f]->", e->v, e->d);
    edge_print(e->next);
  }
  else printf(">\n");
}

void testDijkstra() {
  EdgeList *vis[5] = {0, 0, 0, 0, 0};
  IntDist *d = 0;
  IntList *path = 0;
  int i;

  vis[0] = edge_insert(edge_insert(edge_insert(vis[0], 1, 3), 2, 5), 3, 1);
  vis[1] = edge_insert(edge_insert(vis[1], 0, 3), 2, 2);
  vis[2] = edge_insert(edge_insert(edge_insert(vis[2], 0, 5), 1, 2), 4, 1);
  vis[3] = edge_insert(edge_insert(vis[3], 0, 1), 4, 1);
  vis[4] = edge_insert(edge_insert(vis[4], 3, 1), 2, 1);

  d = dijkstra(vis, 5, 0);

  for(i=1; i<5; i++) {
    path = d_to_path(d, i);
    int_print(path);
  }
}

#define CLOSE(x,y) = (abs((x)-(y)) < 0.0001)

int intersect(Point p, Point q, Point u, Point v) {
  double den;
  double ua, ub;

  den = (v.y-u.y)*(q.x-p.x) - (v.x-u.x)*(q.y-p.y);
  if(CLOSE(den, 0))
    return 0; /* coincident or parallel */
    
  ua = (v.x-u.x)*(p.y-u.y) - (v.y-u.y)*(p.x-u.x);
  ua /= den;
  ub = (q.x-p.x)*(p.y-u.y) - (q.y-p.y)*(p.x-u.x);
  ub /= den;
  
  return ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1;
}

int pt_in_obstacle(Point *p, Vertex *u) {
  Vertex *v = u;
  double th;
  do {
    th = angle_between(v->pt, *p) - angle_between(v->pt, v->next->pt);
    th = sfDegToRad(th);
    if(sin(th) > 0) return 0;
    v = v->next;
  } while(v != u);
  return 1;
}

int diag_of(Point p, Point q, Vertex *ob) {
  Vertex *current = ob;
  Vertex *vp = 0, *vq = 0;
  do {
    /* look for vertices that match the points */
    if(current->pt.x == p.x && current->pt.y == p.y)
      vp = current;
    else if(current->pt.x == q.x && current->pt.y == q.y)
      vq = current;
  } while(current != ob);
  if(vp && vq) {
    /* see if they are successive corners */
    if(vp->next == vq || vq->next == vp)
      return 0;
    else return 1;
  }
  else return 0;
}

/* we create the vertices and vis arrays */
EdgeList **visibility(Point *start, Point *goal, Vertex **grown, int obs,
		      int *vnum) {
  int i, j, k;
  Vertex *u;
  Point *p, *q;
  EdgeList **vis;
  double d;

  int num = 2;

  /* count the vertices and populate the vertices array */
  for(i = 0; i < obs; i++) {
    u = grown[i];
    do { num++; u = u->next; } while(u != grown[i]);
  }
  vertices = malloc(num * sizeof(Point));
  j = 0;
  for(i = 0; i < obs; i++) {
    u = grown[i];
    do {
      vertices[j].x = u->pt.x; vertices[j++].y = u->pt.y;
      u = u->next;
    } while(u != grown[i]);
  }
  vertices[num - 2] = *start;
  vertices[num - 1] = *goal;
  
  vis = malloc(num * sizeof(EdgeList *));
  for(i = 0; i < num; i++) vis[i] = 0;

  p = vertices;
  /* for each pair of points */
  for(i = 0; i < num; i++, p++) {
    q = vertices;
    for(j = 0; j < i; j++, q++) {
      /* for each obstacle */
      for(k = 0; k < obs; k++) {
	u = grown[k];
	/* check for endpoints inside object */
	if(pt_in_obstacle(p, u))
	  goto next_vertex;
	if(pt_in_obstacle(q, u))
	  goto next_edge;
	/* check for intersecting edge */
	do {
	  if(intersect(*p, *q, u->pt, u->next->pt))
	    goto next_edge;
	  u = u->next;
	} while(u != grown[k]);
	/* check for non-successive corners in object */
	if(diag_of(*p,*q,u))
	  goto next_edge;
	/* edge okay, add it */
	d = distance_between(*p, *q);
	vis[i] = edge_insert(vis[i], j, d);
	vis[j] = edge_insert(vis[j], i, d);
      }
    next_edge:
    }
  next_vertex:
  }
  
  *vnum = num;
  return vis;
}

/* output the required files for plotting in gnuplot */
/* note that ungrown obstacles are already available */
void make_plots(Point *start, Point *goal, Point *vertices, Vertex **grown,
		int obs, EdgeList **vis, int vrt, IntList *path) {
  FILE *file;
  Vertex *corner;
  EdgeList *edge;
  IntList *vertex;
  int i;

  file = fopen("start_pt.txt", "w");
  fprintf(file, "%f %f", start->x, start->y);
  fclose(file);

  file = fopen("goal_pt.txt", "w");
  fprintf(file, "%f %f", goal->x, goal->y);
  fclose(file);

  file = fopen("grown_obstacles.txt","w");
  for(i = 0; i < obs; i++) {
    corner = grown[i];
    do {
      fprintf(file, "%f %f\n", corner->pt.x, corner->pt.y);
      corner = corner->next;
    } while(corner != grown[i]);
    fprintf(file, "%f %f\n", grown[i]->pt.x, grown[i]->pt.y);
  }
  fclose(file);

  file = fopen("visibility_graph.txt", "w");
  for(i = 0; i < vrt; i++) {
    edge = vis[i];
    while(edge) {
      fprintf(file, "%f %f\n%f %f\n\n", vertices[i].x, vertices[i].y,
	      vertices[edge->v].x, vertices[edge->v].y);
      edge = edge->next;
    }
  }
  fclose(file);

  file = fopen("shortest_path.txt", "w");
  vertex = path;
  while(vertex) {
    fprintf(file, "%f %f\n", vertices[vertex->v].x, vertices[vertex->v].y);
    vertex = vertex->next;
  }
  fclose(file);
}

void test_plots() {
  Point *start = 0;
  Point *goal = 0;
  Point vertices[5];
  Vertex *grown[1] = { 0 };
  int obs = 1;
  EdgeList *vis[5] = { 0, 0, 0, 0, 0 };
  int vrt = 5;
  IntList *path = malloc(sizeof(IntList));

  vertices[0].x = 0; vertices[0].y = 0;
  vertices[1].x = 1; vertices[1].y = 4;
  vertices[2].x = 2; vertices[2].y = 1;
  vertices[3].x = 5; vertices[3].y = 4;
  vertices[4].x = 6; vertices[4].y = 6;

  start = &vertices[0];
  goal = &vertices[4];

  grown[0] = malloc(sizeof(Vertex));
  grown[0]->next = malloc(sizeof(Vertex));
  grown[0]->next->next = malloc(sizeof(Vertex));
  grown[0]->next->next->next = grown[0];
  grown[0]->pt.x = 1; grown[0]->pt.y = 4;
  grown[0]->next->pt.x = 2; grown[0]->next->pt.y = 1;
  grown[0]->next->next->pt.x = 5; grown[0]->next->next->pt.y = 4;

  vis[0] = edge_insert(edge_insert(vis[0], 1, 0), 2, 0);
  vis[1] = edge_insert(edge_insert(edge_insert(edge_insert(vis[1], 0, 0),
					       2, 0), 3, 0), 4, 0);
  vis[2] = edge_insert(edge_insert(edge_insert(vis[2], 0, 0), 1, 0), 3, 0);
  vis[3] = edge_insert(edge_insert(edge_insert(vis[3], 1, 0), 2, 0), 4, 0);
  vis[4] = edge_insert(edge_insert(vis[4], 1, 0), 3, 0);

  path->v = 0;
  int_append(path, 2);
  int_append(path, 3);
  int_append(path, 4);

  make_plots(start, goal, vertices, grown, obs, vis, vrt, path);
}

#define RADIUS 300.0

IntList *last;

void fast_follow(void) {
  Point pos = get_robot_position();
  int side;
  int th; int d;

  /* special case when on last vertex */
  if(!current->next) {
    sfSetHeading(angle_between(pos, vertices[current->v]) - 90);
    sfSetPosition(distance_between(pos, vertices[current->v]));
    return;
  }

  /* we start looking at the next point when we are within
     RADIUS of the current one */
  if(distance_between(pos, vertices[current->v]) < RADIUS) {
    last = current;
    current = current->next;
    if(!current->next) return;
  }

  th = angle_between(pos, vertices[current->v]) - 90;
  d = distance_between(pos, vertices[current->v]);

  /* look at next corner to adjust heading */
  side = angle_between(vertices[last->v], vertices[current->v]) -
    angle_between(vertices[current->v], vertices[current->next->v]);
  if(side < 0) /* left turn */
    th -= asin(RADIUS/d);
  else if(side > 0) /* right turn */
    th += asin(RADIUS/d);
  /* else do nothing (straight) */

  sfSetHeading(th);
  sfSetPosition(d);
}

#define SONAR_THRESH 400
#define AVOIDING 20
#define GO_LEFT 24
#define GO_RIGHT 28
#define H_RADIUS 260
#define V_RADIUS 275

void avoid_follow(void) {
  int d_front;
  
  if(process_state < AVOIDING) {
    fast_follow();
    
    d_front = sfOccPlane(sfFRONT, sfALL, V_RADIUS, H_RADIUS, -H_RADIUS);
    if(d_front < SONAR_THRESH) {
      /* something in our way */
      sfSetPosition(0);
      process_state = AVOIDING;
      printf("something in the way...\n");
    }
  }
  else {
    if(process_state == GO_RIGHT) {
      if(sfOccPlane(sfLEFT, sfALL, H_RADIUS, -V_RADIUS, V_RADIUS)
	 > SONAR_THRESH)
	process_state = sfINIT;
      else
	if(sfOccPlane(sfFRONT, sfALL, V_RADIUS, H_RADIUS, -H_RADIUS)
	   > SONAR_THRESH)
	  sfSetVelocity(100);
	else
	  sfSetHeading(-15);
      return;
    }
    if(process_state == GO_LEFT) {
      if(sfOccPlane(sfRIGHT, sfALL, H_RADIUS, V_RADIUS, -V_RADIUS)
	 > SONAR_THRESH)
	process_state = sfINIT;
      else
	if(sfOccPlane(sfFRONT, sfALL, V_RADIUS, H_RADIUS, -H_RADIUS)
	   > SONAR_THRESH)
	  sfSetVelocity(100);
	else
	  sfSetHeading(15);
	sfSetVelocity(100);
      return;
    }
    if(sfOccPlane(sfRIGHT, sfALL, H_RADIUS, V_RADIUS, -V_RADIUS)
       < sfOccPlane(sfLEFT, sfALL, H_RADIUS, -V_RADIUS, V_RADIUS)) {
      /* go right */
      sfSetDHeading(-90);
      process_state = GO_RIGHT;
    }
    else {
      /* go left */
      sfSetDHeading(90);
      process_state = GO_LEFT;
    }
  }
}

#define EPS_TH 35

int angle_diff(int th1, int th2) {
  while(th1 < 0) th1 += 360;
  while(th2 < 0) th2 += 360;
  th1 %= 360;
  th2 %= 360;
  return th1 > th2 ? th1 - th2 : th2 - th1;
}

void best_avoid(void) {
  Point pos = get_robot_position();
  int side;
  int th; int d;
  double max; int i, j; float x, y; Point p;

  if(sfStalledMotor(sfLEFT) || sfStalledMotor(sfRIGHT) &&
     process_state != sfINIT) {
    sfSetPosition(-200);
    process_state = -5;
    return;
  }

  process_state = 20;

  /* special case when on last vertex */
  if(!current->next) {
    sfSetHeading(angle_between(pos, vertices[current->v]) - 90);
    sfSetPosition(distance_between(pos, vertices[current->v]));
    return;
  }

  /* we start looking at the next point when we are within
     RADIUS of the current one */
  if(distance_between(pos, vertices[current->v]) < RADIUS) {
    last = current;
    current = current->next;
    if(!current->next) return;
  }

  th = angle_between(pos, vertices[current->v]) - 90;
  d = distance_between(pos, vertices[current->v]);

  /* look at next corner to adjust heading */
  side = angle_between(vertices[last->v], vertices[current->v]) -
    angle_between(vertices[current->v], vertices[current->next->v]);
  if(side < 0) /* left turn */
    th -= asin(RADIUS/d);
  else if(side > 0) /* right turn */
    th += asin(RADIUS/d);
  /* else do nothing (straight) */

  /* check how close th is to our actual heading */
  if(abs(sfRobot.ath - th) < EPS_TH) {
    printf("%d -> ", th);
    /* use sensors to improve heading */
    max = 0;
    for(i = 0; i < CBUF_LEN; i++) {
      if(!sraw_buf->valid[i]) continue;
      x = sraw_buf->xbuf[i];
      y = sraw_buf->ybuf[i];
      p.x = x + 0.5;
      p.y = y + 0.5;
      if(distance_between(pos, p) > max &&
	 angle_diff(angle_between(pos, p)+90, th) < EPS_TH) {
	max = distance_between(pos, p);
	th = angle_between(pos, p)+90;
      }
    }
    printf("%d\n", th);
  }

  sfSetHeading(th);
  sfSetPosition(d);
}

void myStartup(void)
{
  sfSetDisplayState(sfGLOBAL, TRUE);
  sfInitProcess(sfRunEvaluator, "evaluator");
}

void myConnect(void)
{
  Point start, goal;
  Vertex *obs[MAX_POLY];
  EdgeList **vis;
  IntDist *d;
  int num, i;

  sfSetMaxVelocity(600);

  printf("st go\n");
  read_start("/home/tps12/homework/robotics/hw5/start.txt");
  read_goal("/home/tps12/homework/robotics/hw5/goal.txt");
  sg_points(&start, &goal);

  printf("obs\n");
  read_obstacles("/home/tps12/homework/robotics/hw5/obstacles.txt");
  for(i = 0; i < ct; i++)
    obs[i] = polygrow(obstacles[i], DEFAULT_DIAMETER);

  printf("visi\n");
  vis = visibility(&start, &goal, obs, ct, &num);
  printf("dijk\n");
  d = dijkstra(vis, num, num-2);
  printf("path\n");
  path = d_to_path(d, num-1);
  printf("plots\n");
  make_plots(&start, &goal, vertices, obs, ct, vis, num, path);
  exit(0);

  sfInitProcess(follow_points, "avoid_follow");
}

#ifdef IS_UNIX
void main(int argc, char **argv)
#endif
#ifdef MS_WINDOWS
int PASCAL
WinMain (HANDLE hInst, HANDLE hPrevInstance, LPSTR lpszCmdLine, int nCmdShow)
#endif
{
  /*  test_plots();
  testDijkstra();
  return;*/
  sfOnConnectFn(myConnect);	/* register a connection function */
  sfOnStartupFn(myStartup);	/* register a startup function */
#ifdef IS_UNIX
  sfStartup(0);			/* start up the Saphira window */
#endif
#ifdef MS_WINDOWS
  sfStartup(hInst, nCmdShow, 0); 
  return 0;
#endif
}



