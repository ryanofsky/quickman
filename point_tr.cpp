/*
                                     - Dr. Wily  */

////////////////////////////////////////////////////////// INITIAL DECLARATIONS

#ifdef RUSS
#define INPATH "m:/russ/My Documents/quickman/"
#define OUTPATH "m:/russ/My Documents/quickman/out/"
#else
#define INPATH
#define OUTPATH
#endif


#include "saphira.h"
#include "world.h"
#include "general.h"
#include "point.h"

typedef void follower(void);
typedef Point<float> RPoint;

////////////////////////////////////////////////////////////// global variables

World world;
Point<float> initialPosition(4235, 475);
World::PathIterator current(world);
World::PathIterator last(world);

///////////////////////////////////////////////////////////// follower functions

// There are four different follower functions

follower best_avoid;    // default one
follower follow_points; // stop and go follow algorithm
follower avoid_follow;  // obstacle avoider
follower fast_follow;   // speedy

// Set this to change which follower to use
follower * WHICHFOLLOWER = best_avoid;

////////////////////////////////////////////////////////////////// program main

void myStartup(void);
void myConnect(void);

#ifdef IS_UNIX
void main(int argc, char **argv)
#endif
#ifdef MS_WINDOWS
#ifdef MS_CONSOLE
int main()
#else
int WINAPI WinMain( HINSTANCE hInst, HINSTANCE hPreInst, LPSTR lpszCmdLine, int nCmdShow )
#endif
#endif
{
  try
  { 
    CFile obstacle(INPATH "obstacle.txt", "r");
    world.readFile(obstacle,world.vertices,&world.shapes);

    CFile start(INPATH "start.txt","r");
    world.readFile(start,world.startarea);

    CFile goal(INPATH "goal.txt","r");
    world.readFile(goal,world.goalarea);


    // GROW METHOD #1:
    world.growShapes(1.6);
    
    
    // GROW METHOD #2 (fast)
    // the second argument is the diameter of the robot
    // calculated from "370 x 550  mm."
    // current value is too conservative for our obstacle course
 
    //world.fgrowShapes(662.87253676706202790366913417296);    
  
    CFile grown(OUTPATH "grown.txt","w");
    world.outputShapes(grown, world.vertices.begin(), world.vertices.end());
    world.outputShapes(grown, world.gvertices.begin(), world.gvertices.end());

    world.makeVisibility();
    CFile visibility(OUTPATH "visibility.txt","w");

    world.findPath();
 
    CFile thepath(OUTPATH "path.txt","w");
    world.outputPath(thepath);
  }
  catch(SimpleException e)  
  {
    e.print();
  }
  catch(...)
  {
    cerr << "Unknown exception caught" << endl;
  }
 
  sfOnConnectFn(myConnect);     /* register a connection function */
  sfOnStartupFn(myStartup);     /* register a startup function */
#ifdef IS_UNIX
  sfStartup(0);                 /* start up the Saphira window */
#endif
#ifdef MS_WINDOWS
#ifndef MS_CONSOLE
  sfStartup(hInst, nCmdShow, 0); 
#endif  
  return 0;
#endif
};

// helper functions

RPoint get_robot_position() {
  return RPoint(initialPosition.x - sfRobot.ay, initialPosition.y + sfRobot.ax);
}

int angle_between(RPoint p1, RPoint p2) {
  return (int)(sfRadToDeg(atan2(p2.y - p1.y, p2.x - p1.x)));
}

int distance_between(RPoint p1, RPoint p2) {
  return (int)p1.distanceTo(p2);
}

void draw_path()
{
  sfSetLineWidth (5);
  sfSetLineType (sfLINESOLID);
  sfSetLineColor (sfColorBrickRed);

  for(int i = world.path.size() - 2; i >= 0; --i)
  {
    World::GVertex p = world.get_node(world.path[i+1]);
    World::GVertex q = world.get_node(world.path[i]);
    //sfSMessage("drawing line from (%i, %i) to (%i, %i)", p.x, p.y, q.x, q.y);
    sfDrawVector(
      p.y - initialPosition.y,
      initialPosition.x - p.x,
      q.y - initialPosition.y,
      initialPosition.x - q.x
    );
  }
  sfSetLineWidth (1);
}

void myStartup(void)
{
  sfSetDisplayState(sfGLOBAL, TRUE);
  //sfInitProcess(sfRunEvaluator, "evaluator");
}

void myConnect(void)
{
  world.start = get_robot_position();
  world.reorient();
  world.findPath();

  CFile visibility(OUTPATH "nvisibility.txt","w");
  world.outputVisibility(visibility);

  CFile thepath(OUTPATH "npath.txt","w");
  world.outputPath(thepath);

  if (world.path.size() < 2)
    sfMessage("I am a stupid robot. I do not know where to go.");
 
  current = last = world.path.end();
  --current;

  sfSetMaxVelocity(600);
  sfInitProcess(WHICHFOLLOWER, "avoid_follow");  
}


// followers

void best_avoid(void) {

  draw_path();

  // constants
  const int RADIUS = 200;
  const int MOVING = 20;
  const int PBUFFERSIZE = 5;
  
  // variables
  RPoint pos = get_robot_position();

  int side;
  int th; int d;
  World::WPoint p; double amt;
  int vel;

  if((sfStalledMotor(sfLEFT) || sfStalledMotor(sfRIGHT))
     && process_state != sfINIT) {
    sfSetPosition(-200);
    process_state = -5;
    return;
  }

  process_state = 20;

  /* special case when on last vertex */
  if(current.isLast()) {
    sfSMessage("Going to the last point (%i, %i)", current->x, current->y);
    sfSetHeading(angle_between(pos,*current) - 90);
    sfSetPosition(distance_between(pos, *current));
    
    if (sfDonePosition(100))
    {
      sfSMessage("Finished");
      //sfDisconnectFromRobot();
      process_state=sfSUCCESS;
    };
    return;
  }

  /* we start looking at the next point when we are within
     RADIUS of the current one */
  if(distance_between(pos, *current) < RADIUS) {
    last = current;
    sfSMessage("Finished with point #%i (%i, %i)", current.number(), current->x, current->y);
    --current;
    if(current.isLast())
      return;
  }

  th = angle_between(pos, *current) - 90;
  d = distance_between(pos,*current);

  vel = (int) (600 * cos(2*(sfRadToDeg(abs((int)(th-sfRobot.ath))))));
  if(vel < 300) vel = 300;
  sfSetMaxVelocity(vel);

  /* look at next corner to adjust heading, maybe */
  side = angle_between(*last, *current) -
    angle_between(*current, current.next());
  if(side < 0) /* left turn */
    amt = -asin(RADIUS/d);
  else if(side > 0) /* right turn */
    amt = asin(RADIUS/d);
  /* else do nothing (straight) */
  else amt = 0;
  amt *= d < 600 ? (double)d/600 : 1;
  th += (int)amt;

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

  Point<float> ob;
  float ox, oy;
  
  /* look for unexpected obstacles */
  if(sfOccPlaneRet(sfFRONT, sfFRONT, 400, 200, -200, &ox, &oy) < 500) {
    /* a new obstacle found! */
    ob.x = pos.x - oy;
    ob.y = pos.y + ox;
    sfSMessage("obstacle %f %f!\n", ob.x, ob.y);
  }
/*   else */
/*     printf("%d\n", sfOccPlaneRet(sfFRONT, sfFRONT, 250, 150, -150, ox, oy)); */

  sfSetHeading(th);
  sfSetPosition(d);
}

void follow_points(void) {
  const int DONE_MOVING = 20;
  const int ROTATING    = 24;
  const int MOVING      = 28;
  const int GOAL        = 32;

  World::WPoint robot_pos = get_robot_position();

  draw_path();

  switch(process_state) {
  case sfINIT:
    sfSMessage("I'm Initializing");
    process_state = MOVING;
    break;
  case ROTATING:
    if (sfDoneHeading(10)) {
      int distance = distance_between(robot_pos, *current);
      sfSMessage("Finished rotating. Moving %i mm to (%i, %i)", distance, current->x, current->y);
      sfSetPosition(distance);
      process_state = MOVING;
    }
    break;
  case MOVING:
    if(sfDonePosition(100)) {
      sfSMessage("Finished moving to (%i, %i)", current->x, current->y);
      if (current.isLast())
        process_state = GOAL;
      else { 
        --current;
        int angle = angle_between(robot_pos, *current)- 90;
        sfSMessage("Pointing by %i degrees to (%i, %i)", angle, current->x, current->y);
        sfSetHeading(angle);
        process_state = ROTATING;       
      }
    }
    break;
  case GOAL:
    sfSMessage("I am at the goal. Give me prizes.");
    sfSuspendSelf(10);
    break;
  }
}

void fast_follow(void) {
  const double RADIUS = 300.0;

  World::WPoint pos = get_robot_position();
  int side;
  int th; int d;

  draw_path();

  /* special case when on last vertex */
  if(current.isLast()) {
    sfSetHeading(angle_between(pos, *current) - 90);
    sfSetPosition(distance_between(pos, *current));
    return;
  }

  /* we start looking at the next point when we are within
     RADIUS of the current one */
  if(distance_between(pos, *current) < RADIUS) {
    last = current;
    --current;
    if(current.isLast()) return;
  }

  th = angle_between(pos, *current) - 90;
  d = distance_between(pos, *current);

  /* look at next corner to adjust heading */
  side = angle_between(*last,*current) -
    angle_between(*current, current.next());
  if(side < 0) /* left turn */
    th -= (int)asin(RADIUS/d);
  else if(side > 0) /* right turn */
    th += (int)asin(RADIUS/d);
  /* else do nothing (straight) */

  sfSetHeading(th);
  sfSetPosition(d);
}



void avoid_follow(void) {

  const int SONAR_THRESH = 400;
  const int AVOIDING = 20;
  const int GO_LEFT = 24;
  const int GO_RIGHT = 28;
  const int H_RADIUS = 260;
  const int V_RADIUS = 275;

  int d_front;
  
  if(process_state < AVOIDING) {
    fast_follow();
    
    d_front = sfOccPlane(sfFRONT, sfALL, V_RADIUS, H_RADIUS, -H_RADIUS);
    if(d_front < SONAR_THRESH) {
      /* something in our way */
      sfSetPosition(0);
      process_state = AVOIDING;
      sfSMessage("something in the way...\n");
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


