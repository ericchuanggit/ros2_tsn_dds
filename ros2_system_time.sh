#!/bin/bash
source install/setup.bash

function print_help() {
	echo -e "Usage:"
	echo -e "  $0 <exactly one Option>"
	echo -e "Options:"
	echo -e "  -h, --help       Display this help"
	echo -e "  -t, --talker     Start ROS2 publish system time"
	echo -e "  -l, --listener   Start ROS2 listene system time"
}

ROS_PREFIX=${AMENT_PREFIX_PATH}
PKG_NAME="ros2_pub_sub"

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
	-t | --talker )
		application=sender_time
		;;
	-l | --listener )
		application=receiver_time
		;;
	* )
		echo "Invalid argument $1"
		print_help
		return 1;
		;;
esac

profile=./dds_profile/dds_${application}_profile.xml

white_list=$(awk -F '[<>]' '/address/{print $3}' $profile)

if ! $(ip addr show | grep -q ${white_list}); then
	echo "Whitelist IP $white_list not configured"
	echo "Please run net_setup.sh first"
	return 1
fi

export NDDS_QOS_PROFILES=$profile
export FASTRTPS_DEFAULT_PROFILES_FILE=$profile

echo "Ros2 $application is now running. do CTRL-C to exit"
sleep 1

ros2 run ${PKG_NAME} $application
