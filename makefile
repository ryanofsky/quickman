#
# Makefile for quickman, 
# mostly copied from /usr/local/saphira/handler/src/apps/makefile
#
# Before running this makefile do: . /usr/local/saphira/setup.sh 

SHELL = /bin/sh

#############################################################

SRCD = ./
OBJD = ./
INCD = $(SAPHIRA)/handler/include/
LIBD = $(SAPHIRA)/handler/obj/
BIND = ./
COLBERT = $(SAPHIRA)/colbert/

# find out which OS we have 
include $(SAPHIRA)/handler/include/os.h

CFLAGS =  -g -D$(CONFIG) $(PICFLAG) $(REENTRANT)
CC = gcc
CPP = g++
INCLUDE = -I$(INCD) -I$(X11D)include


#############################################################
all: $(BIND)quickman
	touch all

$(OBJD)point_tr.o: $(SRCD)point_tr.cpp $(INCD)saphira.h $(SRCD)point.h  $(SRCD)qman.h  $(SRCD)smatrix.h  $(SRCD)world.h
	$(CPP) $(CFLAGS) -c $(SRCD)point_tr.cpp $(INCLUDE) -o $(OBJD)point_tr.o

$(OBJD)world.o: $(SRCD)world.cpp $(SRCD)point.h  $(SRCD)qman.h  $(SRCD)smatrix.h  $(SRCD)world.h
	$(CPP) $(CFLAGS) -c $(SRCD)world.cpp $(INCLUDE) -o $(OBJD)world.o

$(BIND)quickman: $(OBJD)point_tr.o $(OBJD)world.o
	$(CPP) $(OBJD)point_tr.o $(OBJD)world.o -o $(BIND)quickman \
-L$(LIBD) -lsf -L$(MOTIFD)lib $(LLIBS) -lc -lm 

