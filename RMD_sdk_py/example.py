from RMD_main_Func import *

# initialise the RMD class with CAN Port, Baudrate and motor ID.
motor1 = RMD('can1',1000000,1)

# Reading the motor status
motor1.RD_Motor_Status_2()
# Send and receive the CAN Frames to perform the above operation
motor1.Send_Receive_Message()
# decode and print the message received from the motor
motor1.decode_mesgrecv()
