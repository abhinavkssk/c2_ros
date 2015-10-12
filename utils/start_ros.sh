#!/bin/sh

file="running_proc_id.txt"

if [ -f "$file" ]
then
	./stop_ros.sh "$file"
	sleep 5
fi

echo "starting c2_ros ..."
nohup roslaunch c2_ros c2_simulation.launch >> c2_roslaunch.txt&
echo $! >> $file

sleep 2


#put this at last
echo "starting rosbag -a ..."
nohup roslaunch c2_ros rosbag_record.launch >> rosbag_launch.txt&
echo $! >> $file
