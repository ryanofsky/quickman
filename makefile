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

$(OBJD)point_tr.o: $(SRCD)point_tr.c $(INCD)saphira.h
	$(CC) $(CFLAGS) -c $(SRCD)point_tr.c $(INCLUDE) -o $(OBJD)point_tr.o

$(BIND)quickman: $(OBJD)point_tr.o
	$(CC) $(OBJD)point_tr.o -o $(BIND)quickman \
-L$(LIBD) -lsf -L$(MOTIFD)lib $(LLIBS) -lc -lm 

