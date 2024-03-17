
# This file should be sourced
set -xv
here="$(dirname $(realpath $0))"

export LOCOBOT_IP="$(getent hosts locobot1 |  awk '{ print $1 }')"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${here}/src/interbotix_ros_rovers/interbotix_ros_xslocobots/install/resources/super_client_configuration_file.xml"
export ROS_DISCOVERY_SERVER="${LOCOBOT_IP}:11811"
export INTERBOTIX_XSLOCOBOT_BASE_TYPE=create3


sudo ip route add 192.168.186.0/24 via "${LOCOBOT_IP}"

set +xv

