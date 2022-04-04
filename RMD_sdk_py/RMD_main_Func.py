#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################
# This file have all the functions of the RMD
########################################################

from __future__ import print_function
import can
import RMD_Dcl
from RMD_Dcl import *

class RMD:
    # This function takes the basic CAN setup parameters and setups CAN communication
    def __init__(self, channel, bitrate=1000000, motor_id=0):
        self.mesgsent = None
        self.mesgrecv = None
        self.Temp_mesgrecv = None
        self.Channel_ID = channel
        self.bitrate = bitrate
        self.Motor_ID = motor_id
        self.bus = can.interface.Bus(bustype='socketcan',
                                             channel=self.Channel_ID, bitrate=self.bitrate)
        try:
            if motor_id is 0:
                raise Exception("ID shoudl be greater than 0")
            else:
                self.Motor_ID = motor_id
        except ValueError:
            print("Wrong ID given")

    # Read PID parameter command 0x30
    def RD_PID_Data(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Write PID parameter to RAM command 0x31
    def WR_PID_RAM(self, PositionKP, PositionKi, SpeedKp, SpeedKi, TorqueKp, TorqueKi):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x31, 0x00, PositionKP, PositionKi, SpeedKp, SpeedKi, TorqueKp, TorqueKi], is_extended_id=False)

    # Write PID parameter to ROM command 0x32
    def WR_PID_ROM(self, PositionKP, PositionKi, SpeedKp, SpeedKi, TorqueKp, TorqueKi):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x32, 0x00, PositionKP, PositionKi, SpeedKp, SpeedKi, TorqueKp, TorqueKi], is_extended_id=False)

    # Read acceleration data command 0x33
    def RD_Acceleration_Data(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Write acceleration data to RAM command 0x34
    def WR_Acceleration_Data(self, Acceleration):
        A = Signed_int_to_hex(int(Acceleration*6), 32)[2:]
        B = f"{A:0>8}"
        Acceleration = bytearray.fromhex(B)
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x34, 0x00, 0x00, 0x00, Acceleration[3], Acceleration[2], Acceleration[1], Acceleration[0]], is_extended_id=False)

    # Read encoder data command 0x90
    def RD_Encoder_Data(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Write Encoder Position Command 0x91
    def WR_Encoder_Offset(self, Offset):
        Offset = int(Offset)
        Offset = hex(Offset)[2:]
        Offset = f"{Offset:0>4}"
        Offset = bytearray.fromhex(Offset)
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x91, 0x00, 0x00, 0x00, 0x00, 0x00, Offset[1], Offset[0]], is_extended_id=False)

    # Write current position to ROM as motor zero position command 0x19
    def WR_Current_Position_AS_Offset_ROM(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Read multi turns angle command 0x92
    def RD_Multi_turn_Angle(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Read single circle angle command 0x94
    def RD_Single_Circle_Angle(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Read motor status 1 and error flag commands 0x9A
    def RD_Error_Flag(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Clear motor error flag command 0x9B
    def Clr_Error_flags(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Read motor status 2 0x9C
    def RD_Motor_Status_2(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Read motor status 3 0x9D
    def RD_Motor_Status_3(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x9D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Motor off command 0x80
    def Motor_OFF(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Motor stop command 0x81
    def Motor_Stop(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Motor start command 0x88
    def Motor_Start(self):
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], is_extended_id=False)

    # Torque current control command 0xA1
    def Torque_Ctrl_Mode(self, Current):
        Current = int((Current/32)*2000)
        Current = Signed_int_to_hex(Current, 16)[2:]
        Current = f"{Current:0>4}"
        Current = bytearray.fromhex(Current)
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0xA1, 0x00, 0x00, 0x00, Current[1], Current[0], 0x00, 0x00], is_extended_id=False)

    # Speed control command 0xA2
    def Speed_Ctrl_Mode(self, Speed):
        Speed_dec = int(Speed*216000)
        Speed_Hex = Signed_int_to_hex(Speed_dec, 32)[2:]
        Speed_Hex = f"{Speed_Hex:0>8}"
        Speed_SA1 = bytearray.fromhex(Speed_Hex)
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0xA2, 0x00, 0x00, 0x00, Speed_SA1[3], Speed_SA1[2], Speed_SA1[1], Speed_SA1[0]], is_extended_id=False)

    # Position Control Mode 1 0xA3
    def Position_Ctrl_1(self, Angle):
        Angle_dec = int(Angle*600)
        Angle_Hex = Signed_int_to_hex(Angle_dec, 32)[2:]
        Angle_Hex = f"{Angle_Hex:0>8}"
        Angle = bytearray.fromhex(Angle_Hex)
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0xA3, 0x00, 0x00, 0x00, Angle[3], Angle[2], Angle[1], Angle[0]], is_extended_id=False)

    # Position Control Mode 2 0xA4
    def Position_Ctrl_2(self, Speed, Angle):
        Speed = int(Speed*6)
        Speed = hex(Speed)[2:]
        Speed = f"{Speed:0>4}"
        Speed = bytearray.fromhex(Speed)
        Angle_dec = int(Angle*600)
        Angle_Hex = Signed_int_to_hex(Angle_dec, 32)[2:]
        Angle_Hex = f"{Angle_Hex:0>8}"
        Angle = bytearray.fromhex(Angle_Hex)
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0xA4, 0x00, Speed[1], Speed[0], Angle[3], Angle[2], Angle[1], Angle[0]], is_extended_id=False)

    # Position Control Mode 3 0xA5
    def Position_Ctrl_3(self, Direction, Angle):
        if Direction == 0 or Direction == 1:
            Angle = int(Angle*100)
            Angle = hex(Angle)[2:]
            Angle = f"{Angle:0>4}"
            Angle = bytearray.fromhex(Angle)
            self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                        data=[0xA5, Direction, 0x00, 0x00, Angle[1], Angle[0], 0x00, 0x00], is_extended_id=False)
        else:
            print('Enter the Right Format')

    # Position Control Mode 4 0xA6
    def Position_Ctrl_4(self, Direction, Speed, Angle):
        if Direction == 0 or Direction == 1:
            Speed = int(Speed*6)
            Speed = hex(Speed)[2:]
            Speed = f"{Speed:0>4}"
            Speed = bytearray.fromhex(Speed)
            Angle = int(Angle*100)
            Angle = hex(Angle)[2:]
            Angle = f"{Angle:0>4}"
            Angle = bytearray.fromhex(Angle)
            self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                        data=[0xA6, Direction, Speed[1], Speed[0], Angle[1], Angle[0], 0x00, 0x00], is_extended_id=False)
        else:
            print('Enter the Right Format')
            
    # Position Control Mode 4 0xA7
    def Position_Ctrl_5(self,Angle):
        Angle_dec = int(Angle*600)
        Angle_Hex = Signed_int_to_hex(Angle_dec, 32)[2:]
        Angle_Hex = f"{Angle_Hex:0>8}"
        Angle = bytearray.fromhex(Angle_Hex)
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0xA7, 0x00, 0x00, 0x00, Angle[3], Angle[2], Angle[1], Angle[0]], is_extended_id=False)
        
    # Position Control Mode 4 0xA8
    def Position_Ctrl_6(self, Speed, Angle):
        Speed = int(Speed*6)
        Speed = hex(Speed)[2:]
        Speed = f"{Speed:0>4}"
        Speed = bytearray.fromhex(Speed)
        Angle_dec = int(Angle*600)
        Angle_Hex = Signed_int_to_hex(Angle_dec, 32)[2:]
        Angle_Hex = f"{Angle_Hex:0>8}"
        Angle = bytearray.fromhex(Angle_Hex)
        self.mesgsent = can.Message(arbitration_id=320+self.Motor_ID,
                                    data=[0xA8, 0x00, Speed[1], Speed[0], Angle[3], Angle[2], Angle[1], Angle[0]], is_extended_id=False)

    # Functions to send & receive the data and to check any CAN Communication Error
    def Send_Receive_Message(self):
        try:
            self.bus.send(self.mesgsent)
            print(self.mesgsent)
            while True:    
                self.Temp_mesgrecv = self.bus.recv()
                if self.mesgsent.arbitration_id == self.Temp_mesgrecv.arbitration_id and self.mesgsent.data[0]==self.Temp_mesgrecv.data[0]:
                    self.mesgrecv = self.Temp_mesgrecv
                    print(self.mesgrecv)
                    break
            #print("CHANNEL {}".format(bus.channel_info))
            #print("ID is ", hex(mesgrecv.arbitration_id))
            #print("DLC is", mesgrecv.dlc)
            #print("Received Message is", mesgrecv.data)
        except can.CanError:
            print("Message NOT sent")

    # Function to send the CAN Frame
    def Send_Message(self):
        try:
            self.bus.send(self.mesgsent)
            print(self.mesgsent)
        except can.CanError:
            print("Message NOT sent")

    # Function to receive CAN Frames from bus
    def Receive_Message(self):
        self.mesgrecv = self.bus.recv()
        print(self.mesgrecv)

    # Function to Decode all the received Messages
    def decode_mesgrecv(self):
        Array = [f"{(hex(self.mesgrecv.data[0])[2:]):0>2}", f"{(hex(self.mesgrecv.data[1])[2:]):0>2}", f"{(hex(self.mesgrecv.data[2])[2:]):0>2}", f"{(hex(self.mesgrecv.data[3])[2:]):0>2}",
                 f"{(hex(self.mesgrecv.data[4])[2:]):0>2}", f"{(hex(self.mesgrecv.data[5])[2:]):0>2}", f"{(hex(self.mesgrecv.data[6])[2:]):0>2}", f"{(hex(self.mesgrecv.data[7])[2:]):0>2}"]
        # Decode the Message of the Read PID Data 0x30
        # Decode the Message of PID Data After writing it to RAM 0x31
        # Decode the Message of PID Data After writing it to ROM 0x32
        if (self.mesgrecv.data[0] == 0x30 or self.mesgrecv.data[0] == 0x31 or self.mesgrecv.data[0] == 0x32):
            if self.mesgrecv.data[0] == 0x30:
                print('Reading PID Data From Motor')
            elif self.mesgrecv.data[0] == 0x31:
                print('Wrote PID DATA to RAM')
            else:
                print('Wrote PID DATA to ROM')
            RMD_Dcl.POSITION_LOOP_KP = int(Array[2],16)
            RMD_Dcl.POSITION_LOOP_KI = int(Array[3],16)
            RMD_Dcl.SPEED_LOOP_KP = int(Array[4],16)
            RMD_Dcl.SPEED_LOOP_KI = int(Array[5],16)
            RMD_Dcl.TORQUE_LOOP_KP = int(Array[6],16)
            RMD_Dcl.TORQUE_LOOP_KI = int(Array[7],16)
            print("Position loop Kp", RMD_Dcl.POSITION_LOOP_KP, "\nPosition loop Ki", RMD_Dcl.POSITION_LOOP_KI, "\nSpeed loop Kp", RMD_Dcl.SPEED_LOOP_KP,
                  "\nSpeed loop Ki", RMD_Dcl.SPEED_LOOP_KI, "\nTorque loop Kp", RMD_Dcl.TORQUE_LOOP_KP, "\nTorque loop Ki", RMD_Dcl.TORQUE_LOOP_KI)

        # Decode the Acceleration read Data 0x33
        # Decode the Acceleration Write Data 0x34
        elif (self.mesgrecv.data[0] == 0x33 or self.mesgrecv.data[0] == 0x34):
            if self.mesgrecv.data[0] == 0x33:
                print('Reading Acceleration Data')
            else:
                print('Wrote Acceleration Data')
            A = f'{Array[7]}{Array[6]}{Array[5]}{Array[4]}'
            B = Signed_hex_to_int(A, 32)
            RMD_Dcl.ACCELERATION_IN = B
            RMD_Dcl.ACCELERATION_OUT = B/6
            print('Acceleration is', RMD_Dcl.ACCELERATION_OUT, 'dps/s')

        # Decode the Encoder Read Data 0x90
        elif(self.mesgrecv.data[0] == 0x90):
            print("Encoder Read Data")
            # Encoder Calculation
            A = f'{Array[3]}{Array[2]}'
            B = int(A, 16)
            RMD_Dcl.ENCODER_POSITION = B
            # Encoder Raw Calculation
            A = f'{Array[5]}{Array[4]}'
            B = int(A, 16)
            RMD_Dcl.ENCODER_ORIGINAL_POSITION = B
            # Encoder Offset Calculation
            A = f'{Array[7]}{Array[6]}'
            B = int(A, 16)
            RMD_Dcl.ENCODER_OFFSET_POSITION = B
            print('Encoder Position is', RMD_Dcl.ENCODER_POSITION, '\nEncoder Raw Value is',
                  RMD_Dcl.ENCODER_ORIGINAL_POSITION, '\nEncoder offset is position', RMD_Dcl.ENCODER_OFFSET_POSITION)

        # Decode Write encoder offset command 0x91
        # Decode Write current position to ROM as motor zero position command 0x19
        elif(self.mesgrecv.data[0] == 0x91 or self.mesgrecv.data[0] == 0x19):
            if self.mesgrecv.data[0] == 0x91:
                print('Wrote Encoder Offset')
            else:
                print('Wrote current position to ROM as motor zero position')
            A = f'{Array[7]}{Array[6]}'
            B = int(A, 16)
            RMD_Dcl.ENCODER_OFFSET_POSITION = B
            print('Encoder offset position is',
                  RMD_Dcl.ENCODER_OFFSET_POSITION)

        # Decode Read multi turns angle Message 0x92
        elif self.mesgrecv.data[0] == 0x92:
            A = f'{Array[7]}{Array[6]}{Array[5]}{Array[4]}{Array[3]}{Array[2]}{Array[1]}'
            B = Signed_hex_to_int(A,56)
            RMD_Dcl.MULTI_TURN_ANG_IN = B*0.01
            RMD_Dcl.MULTI_TURN_ANG_OUT = (B*0.01)/6
            RMD_Dcl.MULTI_TURN_REV_IN = (B*0.01)/360
            RMD_Dcl.MULTI_TURN_REV_OUT = ((B*0.01)/6)/360
            print('No of Turns of external Shaft =', RMD_Dcl.MULTI_TURN_ANG_OUT, 'deg or', RMD_Dcl.MULTI_TURN_REV_OUT,
                  '\nNo of Turns of internal Shaft =', RMD_Dcl.MULTI_TURN_ANG_IN, 'deg or', RMD_Dcl.MULTI_TURN_REV_IN)

        # Decode Read single circle angle command 0x94
        elif self.mesgrecv.data[0] == 0x94:
            A = f'{Array[7]}{Array[6]}'
            B = int(A, 16)
            RMD_Dcl.SINGLE_CIRCLE_ANG = B*0.01
            RMD_Dcl.SINGLE_CIRCLE_REV = (B*0.01)/360
            print('single turn Angle is', RMD_Dcl.SINGLE_CIRCLE_ANG, 'Deg', RMD_Dcl.SINGLE_CIRCLE_REV, 'rev')

        # Decode Read motor status 1 and error flag commands 0x9A
        # Decode Message after clearing error Flags 0x9B
        elif (self.mesgrecv.data[0] == 0x9A or self.mesgrecv.data[0] == 0x9B):
            if self.mesgrecv.data[0] == 0x9A:
                print('Error Flag Data')
            else:
                print('Cleared Error Flags')
            # Temperature Calculation
            RMD_Dcl.TEMPERATURE = Signed_hex_to_int(Array[1], 8)
            print('Temperature is',RMD_Dcl.TEMPERATURE)
            # Voltage Calculaion
            A = f'{Array[4]}{Array[3]}'
            B = int(A, 16)
            RMD_Dcl.VOLTAGE = B*0.1
            print('Voltage is',RMD_Dcl.VOLTAGE)
            # Error
            if int(Array[7],16)  == 0:
                RMD_Dcl.ERROR_STATUS = 'Normal'
            elif int(Array[7],16)  == 1:
                RMD_Dcl.ERROR_STATUS = 'Low Voltage Error'
            elif int(Array[7],16)  == 4:
                RMD_Dcl.ERROR_STATUS = 'High Temperature Error'
            elif int(Array[7],16)  == 5:
                RMD_Dcl.ERROR_STATUS = 'Low Voltage & High Temperature Error'
            print('Error state is', RMD_Dcl.ERROR_STATUS)

        # Decode Read Motor Status 3
        elif self.mesgrecv.data[0] == 0x9D:
            print('Reading Motor Status 3')
            # Phase A Current Calculation
            A = f'{Array[3]}{Array[2]}'
            B = Signed_hex_to_int(A, 16)
            RMD_Dcl.PHASE_A_CURRENT = (B/64)
            print('Phase A Current', RMD_Dcl.PHASE_A_CURRENT)
            # Phase B Current Calculation
            A = f'{Array[5]}{Array[4]}'
            B = Signed_hex_to_int(A, 16)
            RMD_Dcl.PHASE_B_CURRENT = (B/64)
            print('Phase B Current', RMD_Dcl.PHASE_B_CURRENT)
            # Phase C Current Calculation
            A = f'{Array[7]}{Array[6]}'
            B = Signed_hex_to_int(A, 16)
            RMD_Dcl.PHASE_C_CURRENT = (B/64)
            print('Phase C Current', RMD_Dcl.PHASE_C_CURRENT)

        # Decode Motor OFF Message 0x80
        elif self.mesgrecv.data[0] == 0x80:
            print('Motor OFF')

        # Decode Motor Stop Message 0x81
        elif self.mesgrecv.data[0] == 0x81:
            RMD_Dcl.MOTOR_STATUS = 'Stopped'
            print('Motor is', RMD_Dcl.MOTOR_STATUS)

        # Decode Motor Start Message 0x88
        elif self.mesgrecv.data[0] == 0X88:
            RMD_Dcl.MOTOR_STATUS = 'Started'
            print('Motor is', RMD_Dcl.MOTOR_STATUS)

        # Decode Read Motor Data 2 Message 0x9C
        # Deocde the Torque Control Mode Received Data 0xA1
        # Deocde the Speed Control Mode Received Data 0xA2
        # Decode the Position Control Mode 1 Received Data 0xA3
        # Decode the Position Control Mode 2 Received Data 0xA4
        # Decode the Position Control Mode 3 Received Data 0xA5
        # Decode the Position Control Mode 4 Received Data 0xA6
        elif (self.mesgrecv.data[0] ==0x9C or self.mesgrecv.data[0] ==0xA1 or self.mesgrecv.data[0] ==0xA2 or self.mesgrecv.data[0] ==0xA3 or self.mesgrecv.data[0] ==0xA4 or self.mesgrecv.data[0] ==0xA5 or self.mesgrecv.data[0] ==0xA6 or self.mesgrecv.data[0] ==0xA7 or self.mesgrecv.data[0] ==0xA8):
            if self.mesgrecv.data[0] == 0x9C:
                print('Read Motor Status 2')
            elif self.mesgrecv.data[0] == 0xA1:
                print('Torque Control Mode')
            elif self.mesgrecv.data[0] == 0xA2:
                print('Speed Control Mode')
            elif self.mesgrecv.data[0] == 0xA3:
                print('Position Control Mode 1')
            elif self.mesgrecv.data[0] == 0xA4:
                print('Position Control Mode 2')
            elif self.mesgrecv.data[0] == 0xA5:
                print('Position Control Mode 3')
            elif self.mesgrecv.data[0] == 0xA6:
                print('Position Control Mode 4')
            elif self.mesgrecv.data[0] == 0xA7:
                print('Position Control Mode 5')
            else:
                print('Position Control Mode 6')
            # Temperature Calculation
            RMD_Dcl.TEMPERATURE = Signed_hex_to_int(Array[1], 8)
            print("Temperature is",RMD_Dcl.TEMPERATURE)
            # Torque Current Calculation
            A = f'{Array[3]}{Array[2]}'
            B = Signed_hex_to_int(A, 16)
            RMD_Dcl.TORQUE_CURRENT = ((B*33)/2048)
            print('Torque Current is',RMD_Dcl.TORQUE_CURRENT)
            # Speed Calculation
            A = f'{Array[5]}{Array[4]}'
            B = Signed_hex_to_int(A, 16)
            RMD_Dcl.SPEED_IN_DPS = B
            RMD_Dcl.SPEED_OUT_DPS = B/6
            RMD_Dcl.SPEED_IN_RPS = B/360
            RMD_Dcl.SPEED_OUT_RPS = (B/6)/360
            print('Speed is', RMD_Dcl.SPEED_OUT_DPS, 'dps', 'or', RMD_Dcl.SPEED_OUT_RPS, 'rps')
            # Encoder Position Calculation
            A = f'{Array[7]}{Array[6]}'
            RMD_Dcl.ENCODER_POSITION = int(A, 16)
            print('Encoder is at position', RMD_Dcl.ENCODER_POSITION)
        else:
            print("Mesg Not Decoded")
