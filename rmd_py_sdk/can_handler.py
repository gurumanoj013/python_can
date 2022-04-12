#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import can

########################################################
# This file provides CAN Enable & Disable functions 
########################################################


class CAN_Handler:
    def __init__(self, bustype='socketcan', channel='can0', bitrate=1000000):
        self.channel = channel
        self.bitrate = bitrate
        self.bustype = bustype
        self.bus = can.interface.Bus(bustype=self.bustype, channel=self.channel, bitrate=self.bitrate)

    # Disable CAN Ports on AGX
    def CAN_OFF(self):
        os.popen('sh /home/manjunath/Documents/iltmimi/rmd_py_sdk/rmd_py_sdk/can_disable.sh')
        print('CAN communication is disconnected')

    # Enable CAN Ports on AGX
    def CAN_ON(self):
        os.popen('sh /home/manjunath/Documents/iltmimi/rmd_py_sdk/rmd_py_sdk/can_enable.sh')
        print('CAN communication is established')
        
    # Function to send the CAN Frame       
    def Send_Message(self, motor_ID=0, send_data=None):
        if motor_ID <=0 or send_data ==None:
            print('Enter the Right ID or data')
        else:
            try:
                motor_ID=int(motor_ID)
                mesgsent =can.Message(arbitration_id=320+motor_ID, data=send_data, is_extended_id=False)
                self.bus.send(mesgsent)
                print(mesgsent)
            except can.CanError:
                print("Message NOT sent due to can bus error")
                
    # Function to receive CAN Frames from bus
    def Receive_Message(self):
        mesgrecv = self.bus.recv()
        print(mesgrecv)
        return mesgrecv
    
    def Send_Receive_Message(self,motor_ID=0, send_data=None):
        if motor_ID <=0 or send_data ==None:
            print('Enter the Right ID or data')
        else:
            try:
                motor_ID=int(motor_ID)
                mesgsent =can.Message(arbitration_id=320+motor_ID, data=send_data, is_extended_id=False)
                self.bus.send(mesgsent)
                print(mesgsent)
                while True:    
                    Temp_mesgrecv = self.Receive_Message()
                    if mesgsent.arbitration_id == Temp_mesgrecv.arbitration_id and mesgsent.data[0]==Temp_mesgrecv.data[0]:
                        mesgrecv = Temp_mesgrecv
                        # print(mesgrecv)
                        return mesgrecv
            except can.CanError:
                print("Message NOT sent due to can bus error")
        