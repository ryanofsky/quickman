/*
                                     - Dr. Wily  */
#define INPATH "M:/russ/My Documents/quickman/"
#define OUTPATH "M:/russ/My Documents/quickman/out/"


#include "saphira.h"
//#include "qman.h"
#include "world.h"

#define KEY(x) (d[heap[x]].d)
#define SWAP(x,y) (x) ^= (y); (y) ^= (x); (x) ^= (y)
#define INFINITY 100000
#define CLOSE(x,y) (abs((x)-(y)) < 0.0001)
#define RADIUS 300.0

#define DONE_MOVING 20
#define ROTATING    24
#define MOVING      28
#define GOAL        32

extern int ct;

// globals


World world;

vector<int> path;
vector<int>::iterator current = path.begin();
vector<int>::iterator last = path.begin();

vector<World::WPoint> vertices;
World::WPoint robot_pos;

World::WPoint get_robot_position() {
  World::WPoint pos;
  pos.x = 4235 - sfRobot.ay;
  pos.y = sfRobot.ax + 475;
  return pos;
}

int angle_between(World::WPoint p1, World::WPoint p2) {
  return (int)(sfRadToDeg(atan2(p2.y - p1.y, p2.x - p1.x)));
}

int distance_between(World::WPoint p1, World::WPoint p2) {
  return (int)sqrt((p2.y - p1.y)*(p2.y - p1.y) + (p2.x - p1.x)*(p2.x - p1.x));
}

void follow_points(void) {
  switch(process_state) {
  case sfINIT:
    process_state = DONE_MOVING;
    break;
  case DONE_MOVING:
    robot_pos.x = 4235 - sfRobot.ay;
    robot_pos.y = sfRobot.ax + 475;
    sfSetHeading(angle_between(robot_pos, vertices[current[0]])- 90);
    process_state = ROTATING;
    break;
  case ROTATING:
    if (sfDoneHeading(10)) {
      robot_pos.x = 4235 - sfRobot.ay;
      robot_pos.y = sfRobot.ax + 475;
      sfSetPosition(distance_between(robot_pos, vertices[*current]));
      process_state = MOVING;
    }
    break;
  case MOVING:
    if(sfDonePosition(100)) {
      ++current;
      if(!current) process_state = GOAL;
      else { /* process_state = DONE_MOVING;*/
        robot_pos.x = 4235 - sfRobot.ay;
        robot_pos.y = sfRobot.ax + 475;
        sfSetHeading(angle_between(robot_pos, vertices[*current])- 90);
        process_state = ROTATING;       
      }
    }
    break;
  case GOAL:
    break;
  }
}

void fast_follow(void) {
  World::WPoint pos = get_robot_position();
  int side;
  int th; int d;

  /* special case when on last vertex */
  if(current + 1 == path.end()) {
    sfSetHeading(angle_between(pos, vertices[*current]) - 90);
    sfSetPosition(distance_between(pos, vertices[*current]));
    return;
  }

  /* we start looking at the next point when we are within
     RADIUS of the current one */
  if(distance_between(pos, vertices[*current]) < RADIUS) {
    last = current;
    ++current;
    if(current + 1 == path.end()) return;
  }

  th = angle_between(pos, vertices[*current]) - 90;
  d = distance_between(pos, vertices[*current]);

  /* look at next corner to adjust heading */
  side = angle_between(vertices[*last], vertices[*current]) -
    angle_between(vertices[*current], vertices[*(current+1)]);
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

#define EPS_TH 25
#define RADIUS 200

void best_avoid(void) {
  World::WPoint pos = get_robot_position();
  int side;
  int th; int d;
  World::WPoint p; double amt;
  World::WPoint ob; float ox, oy; int vel;

  if(sfStalledMotor(sfLEFT) && sfStalledMotor(sfRIGHT) &&
     process_state != sfINIT) {
    sfSetPosition(-200);
    process_state = -5;
    return;
  }

  process_state = 20;

  /* special case when on last vertex */
  if(current + 1 == path.end()) {
    sfSetHeading(angle_between(pos, vertices[*current]) - 90);
    sfSetPosition(distance_between(pos, vertices[*current]));
    return;
  }

  /* we start looking at the next point when we are within
     RADIUS of the current one */
  if(distance_between(pos, vertices[*current]) < RADIUS) {
    last = current;
    ++current;
    if(current + 1 == path.end()) return;
  }

  th = angle_between(pos, vertices[*current]) - 90;
  d = distance_between(pos, vertices[*current]);

  vel = 600 * cos(2*(sfRadToDeg(abs((int)(th-sfRobot.ath)))));
  if(vel < 300) vel = 300;
  sfSetMaxVelocity(vel);

  /* look at next corner to adjust heading, maybe */
  side = angle_between(vertices[*last], vertices[*current]) -
    angle_between(vertices[*current], vertices[*(current+1)]);
  if(side < 0) /* left turn */
    amt = -asin(RADIUS/d);
  else if(side > 0) /* right turn */
    amt = asin(RADIUS/d);
  /* else do nothing (straight) */
  else amt = 0;
  amt *= d < 600 ? (double)d/600 : 1;
  th += amt;

  /* check how close th is to our actual heading */
/*   if(abs(sfRobot.ath - th) < 1000) { */
/*     /\* use sensors to improve heading *\/ */
/*     max = 0; */
/*     for(i = 0; i < CBUF_LEN; i++) { */
/*       if(!sraw_buf->valid[i]) continue; */
/*       x = -sraw_buf->ybuf[i]; */
/*       y = sraw_buf->xbuf[i]; */
/*       p.x = x + 0.5 + pos.x; */
/*       p.y = y + 0.5 + pos.y; */
/*       metric = distance_between(pos, p); */
/*       metric = metric > 1000 ? 1000 : metric; */
/*       metric *= cos(sfDegToRad(2*(angle_between(pos, p)-90-th))); */
/*       if(metric > max) { */
/*      max = metric; */
/*      printf("%d %f %f ", th, cos(sfDegToRad(2*(angle_between(pos, p)-90-th))), max); */
/*      th = angle_between(pos, p) - 90; */
/*      printf("%d\n", th); */
/*       } */
/*     } */
/*   } */

  /* try that again */
/*   if(sfOccPlane(sfFRONT, sfFRONT, 300, 200, 0) < 1000) { */
/*     th -= 10; */
/*     printf("adjusting left\n"); */
/*   } */
/*   if(sfOccPlane(sfFRONT, sfFRONT, 300, 0, -200) < 1000) { */
/*     th += 10; */
/*     printf("adjusting right\n"); */
/*   } */

  /* look for unexpected obstacles */
  if(sfOccPlaneRet(sfFRONT, sfFRONT, 400, 200, -200, &ox, &oy) < 500) {
    /* a new obstacle found! */
    ob.x = pos.x - oy;
    ob.y = pos.y + ox;
    printf("obstacle %f %f!\n", ob.x, ob.y);
  }
/*   else */
/*     printf("%d\n", sfOccPlaneRet(sfFRONT, sfFRONT, 250, 150, -150, ox, oy)); */

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
/*
  World::WPoint start, goal;
  Vertex *obs[MAX_POLY];
  EdgeList **vis;
  IntDist *d;
  int num, i;

  path = malloc(sizeof(IntList));
  path->v = 0;
  path->next = 0;
  for(i = 1; i < 8; i++)
    int_append(path, i);
                       
  current = path;

  sfSetMaxVelocity(600);
*/
/*   printf("st go\n"); */
/*   read_start("/home/tps12/homework/robotics/hw5/start.txt"); */
/*   read_goal("/home/tps12/homework/robotics/hw5/goal.txt"); */
/*   sg_points(&start, &goal); */

/*   printf("obs\n"); */
/*   read_obstacles("/home/tps12/homework/robotics/hw5/obstacles.txt"); */
/*   for(i = 0; i < ct; i++) */
/*     obs[i] = polygrow(obstacles[i], DEFAULT_DIAMETER); */

/*   printf("visi\n"); */
/*   vis = visibility(&start, &goal, obs, ct, &num); */
/*   printf("dijk\n"); */
/*   d = dijkstra(vis, num, num-2); */
/*   printf("path\n"); */
/*   path = d_to_path(d, num-1); */
/*   printf("plots\n"); */
/*   make_plots(&start, &goal, vertices, obs, ct, vis, num, path); */

  sfInitProcess(best_avoid, "avoid_follow");
}

#ifdef IS_UNIX
void main(int argc, char **argv)
#endif
#ifdef MS_WINDOWS
int WINAPI WinMain( HINSTANCE hInst, HINSTANCE hPreInst, LPSTR lpszCmdLine, int nCmdShow )
/*
int PASCAL
WinMain (HANDLE hInst, HANDLE hPrevInstance, LPSTR lpszCmdLine, int nCmdShow)
*/
#endif
{
 
  FILE * fp;
  
  fp = fopen(INPATH "obstacle.txt","r");
  world.readFile(fp,world.vertices,&world.shapes);
  fclose(fp);

  fp = fopen(INPATH "start.txt","r");
  world.readFile(fp,world.startarea);
  fclose(fp);

  fp = fopen(INPATH "goal.txt","r");
  world.readFile(fp,world.goalarea);
  fclose(fp);


  world.growShapes();
  
  fp = fopen(OUTPATH "grow.txt","w");
  world.outputShapes(fp, world.vertices.begin(), world.vertices.end());
  world.outputShapes(fp, world.gvertices.begin(), world.gvertices.end());
 // fclose(fp); 

  world.makeVisibility();
 
//  fp = fopen(OUTPATH "visibility.txt","w");
  world.outputShapes(fp, world.vertices.begin(), world.vertices.end());
  world.outputVisibility(fp);
  fclose(fp);

  world.findPath();
 
  fp = fopen(OUTPATH "path.txt","w");
  world.outputShapes(fp, world.vertices.begin(), world.vertices.end());
  world.outputTargets(fp);
  world.outputPath(fp);
  fclose(fp);
 
  sfOnConnectFn(myConnect);     /* register a connection function */
  sfOnStartupFn(myStartup);     /* register a startup function */
#ifdef IS_UNIX
  sfStartup(0);                 /* start up the Saphira window */
#endif
#ifdef MS_WINDOWS
  sfStartup(hInst, nCmdShow, 0); 
  return 0;
#endif
}



