readme.txt

The program's main function is in point_tr.cpp.

There are instructions in there on different options
that can be changed for the robot.

By default, it reads from

  obstacle.txt
  start.txt
  goal.txt

and outputs to 

  grown.txt
  visibility.txt
  path.txt

To see the path in gnuplot type

plot "obstacle.txt" with linespoints, "path.txt" with linespoints

Visibility

plot "grown.txt" with linespoints, "visibility.txt" with linespoints

