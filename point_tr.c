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

void scan(void);
void move(void);
void move2(void);

sfprocess *move_proc;
sfprocess *scan_proc;

int thresh = 2;

Point vertices[] = { { 4700, 1700 },
		     { 4300, 2600 },
		     { 3000, 2600 },
		     { 3000, 4200 },
		     { 4600, 4400 },
		     { 4700, 6000 } };
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
      else process_state = DONE_MOVING;
    }
    break;
  case GOAL:
    break;
  }
}

void myStartup(void)
{
  sfSetDisplayState(sfGLOBAL, TRUE);
  sfInitProcess(sfRunEvaluator, "evaluator");
}

void myConnect(void)
{
  sfSetMaxVelocity(200);
  path = malloc(sizeof(IntList));
  path->v = -1;
  path->next = 0;
  int_append(path, 0);
  int_append(path, 1);
  int_append(path, 2);
  int_append(path, 3);
  int_append(path, 4);
  int_append(path, 5);
  current = path->next;
  sfInitProcess(follow_points, "follow_points");
}

#ifdef IS_UNIX
void main(int argc, char **argv)
#endif
#ifdef MS_WINDOWS
int PASCAL
WinMain (HANDLE hInst, HANDLE hPrevInstance, LPSTR lpszCmdLine, int nCmdShow)
#endif
{
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



