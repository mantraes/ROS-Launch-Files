#!/bin/bash

# uncomment to run rviz on the Nvidia Jetson TX1
# unset GTK_IM_MODULE



source ~/catkin_ws/devel/setup.bash

clear

#
# Run IMU
#

read -n1 -r -p  "Run Razor IMU? [y/n] " key

if [ "$key" = 'y' ]; then

    xterm -e 'source ~/catkin_ws/devel/setup.bash; roslaunch razor_imu_9dof razor-pub.launch' & 
    sleep 3

else
    printf "\nskipping IMU...\n"
fi
printf "\n"
#
# Run jetson at max performance?
#
# This must be run with sudo privelidges 
read -n1 -r -p  "Enable maximum performance (Jetson)? [y/n] " key

if [ "$key" = 'y' ]; then

    xterm -e sudo ./Bash-Scripts/max_perf
    sleep 1

else
    printf "\nskipping max performance...\n"
fi

#
# Start ZED ?
#
printf "\n"
read -n1 -r -p "Start ZED depth sensor? [y/n] " key

if [ "$key" = 'y' ]; then

    xterm -e 'source ~/catkin_ws/devel/setup.bash; roslaunch zed_wrapper zed_depth.launch' & 
    sleep 5

else
    printf "\nskipping zed depth sensor...\n"
fi

#
# Start vel_cmd_filter ?
#
printf "\n"
read -n1 -r -p "Start Velocity Filter? [y/n] " key

if [ "$key" = 'y' ]; then

    xterm -e 'source ~/catkin_ws/devel/setup.bash; rosrun vel_cmd_filter vel_cmd_filter_node' &
    sleep 1

else
    printf "\nskipping velocity filter...\n"
fi

#
# Start robot_localization?
#
printf "\n"
read -n1 -r -p "Start robot_localiztion? [y/n] " key

if [ "$key" = 'y' ]; then

    xterm -e roslaunch robot_localization robo_loco.launch &
    sleep 2

else
    printf "\nskipping robot_localization...\n"
fi


#
# Start move_base ?
#
printf "\n"
read -n1 -r -p "Start Move_Base? [y/n] " key

if [ "$key" = 'y' ]; then

    xterm -e roslaunch move_base move_base.launch &
    sleep 2

else
    printf "\nskipping Move_Base...\n"
fi


printf "\nDone!\n"
