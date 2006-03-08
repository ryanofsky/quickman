QuickMan
========

CVS repository: [https://russ.yanofsky.org/viewvc.py/quickman](https://russ.yanofsky.org/viewvc.py/quickman)

Quickman was an assignment for a robotics class that I worked on with 2 other
people. The program drives a Pioneer mobile robot from a starting point to an
ending point in a room filled with obstacles, trying to avoid collisions. The
obstacle shapes and positions are given as input, and then the program plans an
optimal path and follows it. Quickman is written in C++ and makes extensive use
of the STL.

Quickman uses a commercial but freely available library called
[Saphira](http://www.ai.sri.com/~konolige/) that comes with a graphical robot
simulator. There are makefiles for GCC under Linux and Solaris and a Metrowerks
Codewarrior project file for windows. It uses features of C++ not supported by
MS Visual C++ 6.
