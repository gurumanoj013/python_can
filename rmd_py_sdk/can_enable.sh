#!/bin/bash
echo mrobotics | sudo -S busybox devmem 0x0c303000 32 0x0000C400
echo mrobotics | sudo -S busybox devmem 0x0c303008 32 0x0000C458
echo mrobotics | sudo -S busybox devmem 0x0c303010 32 0x0000C400
echo mrobotics | sudo -S busybox devmem 0x0c303018 32 0x0000C458
echo mrobotics | sudo -S modprobe can
echo mrobotics | sudo -S modprobe can_raw
echo mrobotics | sudo -S modprobe mttcan
echo mrobotics | sudo -S ip link set can0 type can bitrate 1000000 
echo mrobotics | sudo -S ip link set can1 type can bitrate 1000000
echo mrobotics | sudo -S ip link set up can0
echo mrobotics | sudo -S ip link set up can1 
echo mrobotics | sudo -S ip link set can0 type can restart-ms 100
echo mrobotics | sudo -S ip link set can1 type can restart-ms 100

