#!/bin/bash

#Alows switching between which ROS workspace packages can be used
#Default is ones located in ros_gazebo_python
#GAS 3-27-17
alias sourcegs='source ~/simulation/gs_catkin_ws/devel/setup.bash'
alias sourcejm='source ~/simulation/ros_gazebo_python/devel/setup.bash'

#Alias function for easier use of "find" command
#GAS 4-3-17
where()
{
	# $1 refers to first passed in parameter
	# -z checks it for being an empty string
	if [ -z "$1" ]; then
		echo "Please supply the name of the file you would like to find"
	fi
	
	find / -name "$1" 2>/dev/null
}

#There will allow pulling up gazebo simulations for basicbot GA quicker
#GAS 4-3-17
alias viewga1='GAZEBO_MASTER_URI=http://localhost:11345 gzclient'
alias viewga2='GAZEBO_MASTER_URI=http://localhost:11346 gzclient'
alias viewga3='GAZEBO_MASTER_URI=http://localhost:11347 gzclient'
alias viewga4='GAZEBO_MASTER_URI=http://localhost:11348 gzclient'


