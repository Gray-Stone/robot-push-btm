
# This file should be sourced
set -xv
here="$(dirname $(realpath $0))"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export INTERBOTIX_XSLOCOBOT_BASE_TYPE=create3

source /opt/ros/humble/setup.zsh
source "${here}/install/setup.zsh"

set +xv

