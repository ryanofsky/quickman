#include <stdlib.h>

#define RIGHT_MASK 1
#define LEFT_MASK  2

int get_side(double x0, double y0, double x1, double y1, double px, double py) {
  double x;
  if(x0 == x1) /* vertical */
    if(px < x0) return LEFT_MASK;
    else if(px > x0) return RIGHT_MASK;
    else return LEFT_MASK + RIGHT_MASK;
  if(y0 == y1) /* horizontal */
    if(py < y0) return LEFT_MASK;
    else if(py > y0) return RIGHT_MASK;
    else return LEFT_MASK + RIGHT_MASK;
  /* skewed */
  x = ((x1-x0)/(y1-y0)) * (py-y0) + x0;
  if(px < x) return LEFT_MASK;
  else if(px > x) return RIGHT_MASK;
  else return LEFT_MASK + RIGHT_MASK;
}

int intersect(double x0, double y0, double x1, double y1,
	      double x2, double y2, double x3, double y3) {
  if(get_side(x0, y0, x1, y1, x2, y2) & get_side(x0, y0, x1, y1, x3, y3) ||
     get_side(x2, y2, x3, y3, x0, y0) & get_side(x2, y2, x3, y3, x1, y1))
    return 0;
  else return 1;
}

typedef struct il {
  int v;
  struct il *next;
} int_list;

int_list *int_insert(int_list *head, int v) {
  int_list *new;
  new = (int_list *)malloc(sizeof(int_list));
  new->v = v;
  if(head) {
    new->next = head->next;
    head->next = new;
  } else {
    new->next = NULL;
    head = new;
  }
  return head;
}

void int_link(int_list **list, int u, int v) {
  list[u] = int_insert(list[u], v);
  list[v] = int_insert(list[v], u);
}

typedef int_list path;
typedef int_list adj;

void build_vis_graph(adj **graph, int *x, int *y, int num_vertices,
		     adj **obstacles) {
  int i, j, k;
  int_list *itr;

  /* look at every potential new edge */
  for(i = 0; i < num_vertices; i++) {
    for(j = 0; j < num_vertices; j++) {
      /* don't add 0-length edges */
      if(i == j) goto next_vertex;
      /* if it doesn't intersect an obstacle edge, add it */
      for(k = 0; k < num_vertices - 2; k++) {
	itr = obstacles[k];
	while(itr) {
	  if(intersect(x[i], y[i], x[j], y[j],
		       x[k], y[k], x[itr->v], y[itr->v]))
	    goto next_vertex;
	  itr = itr->next;
	}
      }
      graph[i] = int_insert(graph[i], j);
    next_vertex:
    }
  }
}

void test_build() {
  adj *graph[8] = { NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
  /*  int x[8] = { 0, 2, 2, 3, 3, 5, 0, 5 };
      int y[8] = { 1, 3, 1, 2, 0, 2, 0, 3 }; */
  int x[8] = { 0, 3, 3, 2, 2, 5, 0, 5 };
  int y[8] = { 1, 3, 1, 2, 0, 2, 0, 3 };
  int num_vertices = 8;
  adj *obstacles[6] = { NULL, NULL, NULL, NULL, NULL, NULL };
  adj *itr;
  int i;
  
  int_link(obstacles, 0, 1);
  int_link(obstacles, 1, 2);
  int_link(obstacles, 2, 0);
  int_link(obstacles, 3, 4);
  int_link(obstacles, 4, 5);
  int_link(obstacles, 5, 3);
  
  build_vis_graph(graph, x, y, num_vertices, obstacles);

  for(i = 0; i < num_vertices; i++) {
    itr = graph[i];
    printf("%d: ", i);
    while(itr) {
      printf("%d ", itr->v);
      itr = itr->next;
    }
    printf("\n");
  }
}

int main(int argc, char **argv) {
  test_build();

  return 0;
}

/*
void some_function() {
  int *x;
  int *y;
  int num_vertices;
  adj *obstacles;
  adj *graph;

  read_world(x, y, &num_vertices, obstacles);
  
  grow_obstacles(x, y, obstacles, ...);

  /* allocate adjacency lists *//*
  graph = (adj *)malloc(num_vertices * sizeof(adj));

  build_vis_graph(graph, x, y, num_vertices, obstacles);
}
*/
