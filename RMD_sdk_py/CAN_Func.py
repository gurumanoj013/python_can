#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

########################################################
# This file provides basic Turn ON/OFF CAN functions 
########################################################

# Disable CAN Ports on AGX
def CAN_OFF():
    os.popen('sh /home/agx1/Downloads/can_disable.sh')
    print('CAN communication is disconnected')

# Enable CAN Ports on AGX
def CAN_ON():
    os.popen('sh /home/agx1/Downloads/can_enable.sh')
    print('CAN communication is established')
