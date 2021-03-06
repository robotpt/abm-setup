#!/bin/bash

# Go to the directory of this file -- needed for docker-compose
FILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $FILE_DIR

if [ -z ${1+x} ]; 
then 
	MODE="debug";
else
	MODE=$1;
fi

SERVICE_NAME="abm_interaction"
ROS_MASTER_URI=$ROS_MASTER_URI docker-compose up --build
case "$MODE" in 
	setup )
		echo "Calling setup script"
		ROS_MASTER_URI=$ROS_MASTER_URI docker-compose run $SERVICE_NAME bash -c "source ~/ws/catkin_ws/devel/setup.bash && roslaunch abm_interaction setup_abm_interaction.launch"
		;;
	run | start | up )
		echo "Running in the background"
		ROS_MASTER_URI=$ROS_MASTER_URI docker-compose run -d $SERVICE_NAME bash -c "source ~/ws/catkin_ws/devel/setup.bash && roslaunch abm_interaction qt_abm_interaction.launch"
		;;
	terminal | terminal_debug | t )
		ROS_MASTER_URI=$ROS_MASTER_URI docker-compose run $SERVICE_NAME bash -c "source ~/ws/catkin_ws/devel/setup.bash && bash"
		;;
	stop | kill | down )
		docker-compose down
		;;
	debug )
		echo "Starting debug"
		xhost +local: # share display on ubuntu platforms
		ROS_MASTER_URI=$ROS_MASTER_URI docker-compose run $SERVICE_NAME bash -c "terminator -e \"echo 'Entering Docker Container...' && bash\""
		;;
	* )
		echo "Please enter 'setup', 'run', 'stop', 'terminal', or 'debug'"
		;;
esac
