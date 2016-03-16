#!/bin/bash
clear
service chrony stop
echo "======================================"
echo "Synchronizing system clock with rcu..."
echo "======================================"
ntpdate rcu
sleep 5
ntpdate rcu

service chrony start
sleep 1
echo "======================================"
echo "Checking clock against server..."
echo "======================================"
ntpdate -q rcu
