#!/bin/bash
clear
printf "\nSynchronizing with time.nist.gov...\n"

sudo service chrony stop

sudo ntpdate time.nist.gov

sudo service chrony start

printf "Done"
