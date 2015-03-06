include ../Makefile.conf

# Module name and description
MODULE_NAME = LANE_DETECTION
MODULE_COMMENT = Detection of Street Lanes


LFLAGS += -lparam_interface -lipc -lglobal -llocalize_ackerman_interface \
		  -lmap_server_interface `pkg-config --libs opencv` -lbumblebee_basic_interface

CFLAGS += -Wall `pkg-config --cflags opencv`

DIR 			= ./LaneDetector

SOURCES			= $(wildcard ${DIR}/*.cpp *.cpp)

PUBLIC_INCLUDES = $(wildcard ${DIR}/*.h *.h)

PUBLIC_BINARIES = lane_detection

TARGETS 		= lane_detection

lane_detection: $(patsubst %.cpp, %.o, $(SOURCES))

# rules
include ../Makefile.rules