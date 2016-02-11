#!/bin/bash

# uncomment to run rviz on the Nvidia Jetson TX1
# unset GTK_IM_MODULE



source ~/catkin_ws/devel/setup.bash

clear

#
# Run jetson at max performance?
#
# This must be run with sudo privelidges 
read -n1 -r -p  "Enable maximum performance (Jetson)? [y/n] " key

if [ "$key" = 'y' ]; then

    # turn on fan for safety
    echo "Enabling fan for safety..."
    if [ ! -w /sys/kernel/debug/tegra_fan/target_pwm ] ; then
    echo "Cannot set fan -- exiting..."
    fi
    echo 255 > /sys/kernel/debug/tegra_fan/target_pwm

    echo 0 > /sys/devices/system/cpu/cpuquiet/tegra_cpuquiet/enable
    echo 1 > /sys/kernel/cluster/immediate
    echo 1 > /sys/kernel/cluster/force
    echo G > /sys/kernel/cluster/active
    echo "Cluster: `cat /sys/kernel/cluster/active`"

    # online all CPUs - ignore errors for already-online units
    echo "onlining CPUs: ignore errors..."
    for i in 0 1 2 3 ; do
    echo 1 > /sys/devices/system/cpu/cpu${i}/online
    done
    echo "Online CPUs: `cat /sys/devices/system/cpu/online`"

    # set CPUs to max freq (perf governor not enabled on L4T yet)
    echo userspace > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
    cpumax=`cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies | awk '{print $NF}'`
    echo "${cpumax}" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed
    for i in 0 1 2 3 ; do
    echo "CPU${i}: `cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq`"
    done

    # max GPU clock (should read from debugfs)
    cat /sys/kernel/debug/clock/gbus/max > /sys/kernel/debug/clock/override.gbus/rate
    echo 1 > /sys/kernel/debug/clock/override.gbus/state
    echo "GPU: `cat /sys/kernel/debug/clock/gbus/rate`"

    # max EMC clock (should read from debugfs)
    cat /sys/kernel/debug/clock/emc/max > /sys/kernel/debug/clock/override.emc/rate
    echo 1 > /sys/kernel/debug/clock/override.emc/state
    echo "EMC: `cat /sys/kernel/debug/clock/emc/rate`"

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


    sleep 2

else
    printf "\nskipping robot_localization...\n"
fi

#
# Start rtabmap ?
#
printf "\n"
read -n1 -r -p "Start RTABMAP? [y/n] " key

if [ "$key" = 'y' ]; then

    xterm -e 'source ~/catkin_ws/devel/setup.bash; roslaunch rtabmap_ros rgbd_mapping.launch' &
    sleep 2

else
    printf "\nskipping RTABMAP...\n"
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
