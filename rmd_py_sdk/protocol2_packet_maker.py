#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################
# This file have all the functions of the RMD
########################################################

from __future__ import print_function
import can
from can_handler import *
from rmd_dcl import *


class RMD2(RMD_dcl,CAN_Handler):
    # This function takes the basic CAN setup parameters and setups CAN communication
    def __init__(self, bustype='socketcan', channel='can0', bitrate=1000000):
        super().__init__(bustype,Channel, bitrate)


