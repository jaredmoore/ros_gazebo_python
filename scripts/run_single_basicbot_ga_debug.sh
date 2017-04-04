#!/bin/bash
#This script will start the basicbot_ga setup with the python scripts in individual windows to help with debugging
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
	roslaunch basicbot_ga gazebo_only.launch" &

echo "...Done!"
echo "New instance of 'gzserver' started!"
	
xterm -title "Transporter Node" -e "
	rosrun basicbot_ga basicbot_transporter.py" &

xterm -title "Turn_drive_scan Node" -e "
	rosrun basicbot_ga turn_drive_scan_node.py" &

#strips carriage return from end of path and changes dir
cd $(echo $PATH1 | tr -d '\r')
cd "src/basicbot_ga/test/"

xterm -title "GA (non GA) Node" -e "
	python non_ga_server.py" &
	
echo "Script complete!"
echo "Use 'viewga1' to view progress in gazebo"
