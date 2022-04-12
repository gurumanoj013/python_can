from protocol1_packet_maker import *

# This function converts the torque values into bulk CAN Frames
def Torque_Values(Torque):
    Current = int((Torque/32)*2000)
    Current = Signed_int_to_hex(Current, 16)[2:]
    Current = f"{Current:0>4}"
    Current = bytearray.fromhex(Current)
    return Current[0], Current[1]

# This class is used to Bulk Read and Write Operations
class Multi_Motor_Ctrl:
    # This function is used to get torque values and make a CAN Frame
    def WR_Torque(self, Motor_1_Torque, Motor_2_Torque, Motor_3_Torque, Motor_4_Torque):
        Motor1 = Torque_Values(Motor_1_Torque)
        Motor2 = Torque_Values(Motor_2_Torque)
        Motor3 = Torque_Values(Motor_3_Torque)
        Motor4 = Torque_Values(Motor_4_Torque)
        self.mesgsent = can.Message(arbitration_id=self.Motor_ID,
                                    data=[Motor1[1], Motor1[0], Motor2[1], Motor2[0], Motor3[1], Motor3[0], Motor4[1], Motor4[0]], is_extended_id=False)

    # This function is used to send the bulk message
    def Send_Bulk_mesg(self):
        try:
            self.bus.send(self.mesgsent)
            print(self.mesgsent)
        except can.CanError:
            print("Message NOT sent")

    # This function is used to Read the bulk message
    def Recv_Bulk_mesg(self, Num_Of_Motors):
        i = 0
        while True:
            self.Temp_mesgrecv = self.bus.recv()
            if self.Temp_mesgrecv.arbitration_id == 321 or self.Temp_mesgrecv.arbitration_id == 322 or self.Temp_mesgrecv.arbitration_id == 323 or self.Temp_mesgrecv.arbitration_id == 324:
                self.mesgrecv = self.Temp_mesgrecv
                mesg = self.mesgrecv
                RMD.decode_mesgrecv(self,mesg)
                i += 1
                if i == Num_Of_Motors:
                    break


