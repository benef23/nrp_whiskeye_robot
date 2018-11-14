#!/bin/bash

# find MDK path
export WHISKEYE_SIM_PATH="$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd )"

# make packages available
[[ ":$PYTHONPATH:" != *":$WHISKEYE_SIM_PATH:"* ]] && PYTHONPATH=$WHISKEYE_SIM_PATH:$PYTHONPATH
[[ ":$GAZEBO_RESOURCE_PATH:" != *":$WHISKEYE_SIM_PATH:"* ]] && GAZEBO_RESOURCE_PATH=$WHISKEYE_SIM_PATH:$GAZEBO_RESOURCE_PATH

