#
# Make file for Lane Detector
#
# Author: LI XUANPENG <marshallixp@gmail.com>
# Date: 15/02/2014
#
# Modified by Rodrigo Berriel
# Date: 03/2015

OCVFLAGS 	= `pkg-config --cflags opencv`
OCVLIBS 	= `pkg-config --libs opencv` -lstdc++

CPP 		= g++ -std=c++11

# type of system?
LBITS 		= $(shell /usr/bin/getconf LONG_BIT)
ifeq ($(LBITS),64)
# do 64 bit stuff here, like set some CFLAGS
	SFX = 64
else
	SFX = 32
endif

DIR 		= ./LaneDetector

SRCS		+= $(wildcard main.cpp ${DIR}/*.cpp)

OBJS  		= $(patsubst %.cpp, %.o, $(SRCS))

CXXFLAGS	+= $(OCVFLAGS) -Wall -Wextra -pedantic-errors
LIBS		+= $(OCVLIBS)
BINARY		= detector

all: release

release: $(OBJS)
		$(CPP) $^ $(LIBS) $(CXXFLAGS) -o $(BINARY)

.cpp.o:
		$(CPP) $< $(CXXFLAGS) $(LIBS) -c -o $@

clean:
		rm -f $(DIR)/*.o  $(DIR_SIM)/*.o $(DIR_RCD)/*.o $(BINARY)
