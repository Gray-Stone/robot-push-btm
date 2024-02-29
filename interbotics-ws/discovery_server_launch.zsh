
systemd-run --user --unit "ros_fastdds_discovery" --remain-after-exit  /bin/bash -c "source /opt/ros/iron/setup.sh ; fastdds discovery -i 0"
