#!/bin/bash
source install/setup.bash

function print_help() {
	echo -e "Usage:"
	echo -e "  $0 <exactly one Option>"
	echo -e "Options:"
	echo -e "  -h,  --help       Display this help"
	echo -e "  -c,  --circle_controller     Start ROS2 circular path controller"
	echo -e "  -p,  --Partial_circle        Start ROS2 Partial circular path controller"
	echo -e "  -s,  --stop                  Stop controller"
	echo -e "  -w,  --gazebo_world          Start Gazebo world"
	echo -e "  -wt, --gazebo_top_world      Start Gazebo top world"
	
}

ROS_PREFIX=${AMENT_PREFIX_PATH}
PKG_NAME="circle_car"

if [ "$#" -ne 1 ];
then
	echo "Pass exactly 1 argument"
	print_help
	return
fi

if [ -z ${ROS_VERSION+x} ];
then
	echo "ROS envirnoment not set. Please source <ros installation>/bin/setup.sh"
	echo "typically: "
	echo "		source /opt/ros/humble/setup.bash"
	return
fi

echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_VERSION=$ROS_VERSION"

case $1 in
	-h | --help )
		print_help
		return 0
		;;
	-c | --circle )
		application=circle_launch.py
		;;
	-p | --part_circle )
		application=part_circle_launch.py
		;;
	-s | --stop_controller )
		application=stop_launch.py
		;;
	-w | --gazebo_world )
		application=world_circle_launch.py
		;;
	-wt | --gazebo_world_top )
		application=world_circle_top_launch.py
		;;
	* )
		echo "Invalid argument $1"
		print_help
		return 1;
		;;
esac

echo "Ros2 $application is now running. do CTRL-C to exit"
sleep 1


ros2 launch ${PKG_NAME} $application
