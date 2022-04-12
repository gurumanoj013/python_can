#!/usr/bin/env python
# -*- coding: utf-8 -*-

from protocol1_packet_maker import *
from protocol2_packet_maker import *

motor = RMD()
frame = motor.Motor_OFF()
recv_mesg = motor.Send_Receive_Message(1,frame)
motor.decode_mesgrecv(recv_mesg)

