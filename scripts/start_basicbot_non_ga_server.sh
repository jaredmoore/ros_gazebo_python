#!/bin/bash
#Starts basicbot_ga setup with 4 instances of ROS/Gazebo
#
#GAS 2017-04-03

echo "Script started..."
source "commands.bash"
#where is a custom find function defined in commands.bash
PATH1="$(where ros_gazebo_python)"
#echo "${PATH1}"
echo "Killing all instances of 'gzserver'"

xterm -title "Gazebo - Step World Launcher" -e "
	pkill gzserver;
	sleep 3;
	roslaunch basicbot_ga basicbot_ga.launch" &

echo "...Done!"
echo "New instance of 'gzserver' started!"
	
#strips carriage return from end of path and changes dir
cd $(echo $PATH1 | tr -d '\r')
cd "src/basicbot_ga/test/"

xterm -title "GA (non GA) Node" -e "
	python non_ga_server.py" &
	
echo "Script complete!"
echo "Use 'viewga#' to view progress in gazebo"
echo "Where # = [1-4], corresponding to the specific instance"
