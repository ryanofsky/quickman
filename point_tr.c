/*

                                     - Dr. Wily  */

#include "saphira.h"

#define DONE_MOVING 20
#define ROTATING    24
#define MOVING      28
#define GOAL        32

typedef struct p {
  double x;
  double y;
} Point;

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

void scan(void);
void move(void);
void move2(void);

sfprocess *move_proc;
sfprocess *scan_proc;

int thresh = 2;

Point vertices[] = { { 4234, 475 },
		     { 4400, 1200 },
		     { 4500, 2200 },
		     { 4300, 3200 },
		     { 4300, 3600 },
		     { 4500, 4600 },
		     { 4500, 5900 } };
Point robot_pos;
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
  //rwh not legal in normal c: int heap[num];
  int * heap = malloc(num*sizeof(int));
  
  int heapsize = num;
  int i;

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
  free(heap);
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

#define RADIUS 110.0

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

void myStartup(void)
{
  sfSetDisplayState(sfGLOBAL, TRUE);
  sfInitProcess(sfRunEvaluator, "evaluator");
}

void myConnect(void)
{
  sfSetMaxVelocity(600);
  path = malloc(sizeof(IntList));
  path->v = -1;
  path->next = 0;
  int_append(path, 0);
  int_append(path, 1);
  int_append(path, 2);
  int_append(path, 3);
  int_append(path, 4);
  int_append(path, 5);
  int_append(path, 6);
  current = path->next;
  sfInitProcess(fast_follow, "fast_follow");
}

#ifdef IS_UNIX
void main(int argc, char **argv)
#endif
#ifdef MS_WINDOWS
int PASCAL
WinMain (HINSTANCE hInst, HINSTANCE hPrevInstance, LPSTR lpszCmdLine, int nCmdShow)
#endif
{
  testDijkstra();
  //return;
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



