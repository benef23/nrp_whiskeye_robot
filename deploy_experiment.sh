#!/bin/bash  

# This bash script copies the necessary files to your local NRP storage in order
# for the experiment to be called from the frontend.


# rebuild plugin and move to Gazebo plugins
cd whiskeye_model/whiskeye_robot/src/plugin
make

mv $HOME/.opt/nrpStorage/nrp_whiskeye_robot/whiskeye_model/whiskeye_robot/libwhiskeye_gazebo.so $HBP/GazeboRosPackages/devel/lib/ 	# move plugin	
cp -r $HOME/.opt/nrpStorage/nrp_whiskeye_robot/whiskeye_model/whiskeye_msgs $HBP/GazeboRosPackages/devel/lib/python2.7/dist-packages		# move messages


# Move whiskeye models
mv $HOME/.opt/nrpStorage/1_nrp_sp3_whiskeye_robot/whiskeye_model $HBP/Models/

# create symlinks for models
$HBP/Models/create-symlinks.sh


# add support for sdf to bash

echo 'export GAZEBO_STATIC_INCLUDE_DIRS:INTERNAL=/usr/include/gazebo-7;/usr/include/gazebo-7/gazebo;/usr/include/sdformat-4.4;/usr/include/sdformat-4.4/sdf' >> ~/.bashrc
echo 'export GAZEBO_STATIC_CFLAGS:INTERNAL=-I/usr/include/gazebo-7;-I/usr/include/gazebo-7/gazebo;-I/usr/include/sdformat-4.4;-I/usr/include/sdformat-4.4/sdf' >> ~/.bashrc

echo "Experiment deployed"

