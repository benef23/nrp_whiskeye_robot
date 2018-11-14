
# get CWD
CURDIR=$(shell pwd)

# get ROS release
ROS_RELEASE=kinetic

# configure
SRC_FILES=$(wildcard *.msg)
MSGS=$(subst .msg,,$(SRC_FILES))
H_FILES=$(addprefix ../,$(addsuffix .h,$(MSGS)))
DIR_OUT=.
LIST=$(SRC_FILES:.msg=)
INC=-I whiskeye_msgs:$(CURDIR) -I geometry_msgs:/opt/ros/$(ROS_RELEASE)/share/geometry_msgs/msg -I sensor_msgs:/opt/ros/$(ROS_RELEASE)/share/sensor_msgs/msg -I std_msgs:/opt/ros/$(ROS_RELEASE)/share/std_msgs/msg -I nav_msgs:/opt/ros/$(ROS_RELEASE)/share/nav_msgs/msg

# message update target
.PHONY : default
default : __init__.py $(H_FILES)

__init__.py : *.msg
	@echo build: $@
	@./create_init.sh $(LIST)

# if this is to work, $(CURDIR)/../.. must be on ROS_PACKAGE_PATH
../%.h : %.msg makefile
	@echo build: $@
	@/opt/ros/$(ROS_RELEASE)/lib/gencpp/gen_cpp.py -p whiskeye_msgs -o $(DIR_OUT)/.. $(INC) $<
	@/opt/ros/$(ROS_RELEASE)/lib/genpy/genmsg_py.py -p whiskeye_msgs -o $(DIR_OUT) $(INC) $<

clean :
	rm -f *.py
	rm -f *.pyc
	rm -f ../*.h

