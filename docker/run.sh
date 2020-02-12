#!/bin/bash

# Go to the directory of this file -- needed for docker-compose
FILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $FILE_DIR

# share display on ubuntu platforms
xhost +local:

# build the docker container and run the default command 
# set in the Dockerfile or docker-compose.yaml file
ROS_MASTER_URI=$ROS_MASTER_URI docker-compose up --build
