#!/bin/bash

# uncomment to run rviz on the Nvidia Jetson TX1
# unset GTK_IM_MODULE


# This must be run with sudo privelidges 
source ~/catkin_ws/devel/setup.bash

clear

#
# Run jetson at max performance?
#

read -n1 -r -p -s "Enable maximum performance (Jetson)? [y/n] \n" key

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
    echo "skipping max performance\n"
fi

#
# Start ZED ?
#

read -n1 -r -p -s "Start ZED depth sensor? [y/n] \n" key

if [ "$key" = 'y' ]; then

	xterm -e roslaunch zed_wrapper zed_depth.launch & 
	sleep 5

else
    echo "skipping zed depth sensor\n"
fi

#
# Start vel_cmd_filter ?
#

read -n1 -r -p -s "Start Velocity Filter? [y/n] \n" key

if [ "$key" = 'y' ]; then

	xterm -e rosrun vel_cmd_filter vel_cmd_filter_node &
	sleep 1

else
    echo "skipping velocity filter\n"
fi

#
# Start robot_localization?
#

read -n1 -r -p -s "Start robot_localiztion? [y/n] \n" key

if [ "$key" = 'y' ]; then

	xterm -e roslaunch robot_localization robo_loco.launch &
	sleep 2

else
    echo "skipping robot_localization\n"
fi

#
# Start rtabmap ?
#

read -n1 -r -p -s "Start RTABMAP? [y/n] \n" key

if [ "$key" = 'y' ]; then

	xterm -e roslaunch rtabmap_ros rgbd_mapping.launch &
	sleep 2

else
    echo "skipping RTABMAP\n"
fi

echo "Done!"