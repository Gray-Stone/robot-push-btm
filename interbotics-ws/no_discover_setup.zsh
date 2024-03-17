#! /usr/bin/zsh

# Remeber to source this file. 
# I wrote this file to be zsh for my convenience, just change all instance of zsh to bash.
set -xv
here="$(dirname $(realpath $0))"
# fastrpts was already the default setting anyway.
export INTERBOTIX_XSLOCOBOT_BASE_TYPE=create3

source /opt/ros/humble/setup.zsh
source "${here}/install/setup.zsh"

set +xv
