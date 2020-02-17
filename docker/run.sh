#!/bin/bash

# Go to the directory of this file -- needed for docker-compose
FILE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $FILE_DIR

SERVICE_NAME="abm_interaction"
ROS_MASTER_URI=$ROS_MASTER_URI docker-compose up --build
case "$1" in 
	setup )
		echo "Calling setup script"
		ROS_MASTER_URI=$ROS_MASTER_URI docker-compose run $SERVICE_NAME bash -c "source ~/ws/catkin_ws/devel/setup.bash && roslaunch abm_interaction setup_abm_interaction.launch"
		;;
	run )
		echo "Running in the background"
		ROS_MASTER_URI=$ROS_MASTER_URI docker-compose run -d $SERVICE_NAME bash -c "source ~/ws/catkin_ws/devel/setup.bash && roslaunch abm_interaction qt_abm_interaction.launch"
		;;
	stop )
		read -p "Are you sure that you want to stop the running containers?" -n 1 -r
		echo    # (optional) move to a new line
		if [[ ! $REPLY =~ ^[Yy]$ ]]; then
			docker-compose down
		fi
		ROS_MASTER_URI=$ROS_MASTER_URI docker-compose down
		;;
	debug )
		echo "Starting debug"
		xhost +local: # share display on ubuntu platforms
		ROS_MASTER_URI=$ROS_MASTER_URI docker-compose run $SERVICE_NAME bash -c "terminator -e \"echo 'Entering Docker Container...' && bash\""
		;;
	* )
		echo "Please enter 'setup', 'run', 'stop', or 'debug'"
		;;
esac
