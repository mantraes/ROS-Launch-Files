#!/bin/bash
clear
printf "\nStarting NTP Service...\n"

sudo service ntp start

ntpq -p

sudo ntpd

printf "Done"
