from ctypes import *
import time
import ctypes
import sys, os
from pynput.keyboard import Controller,Key,Listener
from pynput import keyboard
import keyboard
import numpy as np
import msvcrt


from ins_read import *      # inspire_hand

lib = CDLL("./wiseglove/WISEGLOVEU3D.dll")
joints = {
    "Thu_MCP": 1,
    "Thu_IP": 2,
    "Thu_to_Index": 3, 
    "Ind_MCP": 4,
    "Ind_PIP": 5,
    "Ind_DIP": 6,
    "Ind_to_Mid": 7, 
    "Mid_MCP": 8,
    "Mid_PIP": 9,
    "Mid_DIP": 10,
    "Mid_to_Rin": 11, 
    "Rin_MCP": 12,
    "Rin_PIP": 13,
    "Rin_DIP": 14,
    "Rin_to_Pin": 15, 
    "Pin_MCP": 16,
    "Pin_DIP": 17,
    "Thu_CMC": 18 
}
#手指各关节标签：拇指MCP(01),IP(02),拇指食指(03),食指MCP-PIP-DIP(04,05,06),食指中指(07),中指MCP-PIP-DIP(08,09,10),
#               中指环指(11),环指MCP-PIP-DIP(12,13,14),环指小指(15),小指MCP-DIP(16,17),拇指CMC(18)
global data_disp
data_disp = ctypes.c_int
g_pGlove = ctypes.c_bool
num_sensor = ctypes.c_int

snfunc = lib.wgGetSn
snfunc.restype = ctypes.c_bool
snfunc.argtypes = [ctypes.POINTER(ctypes.c_char)]


angle = (c_float*18)()
sensor = (c_ushort*18)()
angle_range = (c_float*18)()

data_disp = 1
timestamp = ctypes.c_uint

lib.wgInit.restype = ctypes.c_bool
#g_pGlove=lib.wgInitManu(3,115200)

################################################ data glove init ################################################
g_pGlove=lib.wgInit(1)  # 0=左手,1=右手,-1=不分左右 

if g_pGlove == False:
    print("No Glove")
    lib.wgClose()

else:
    lib.wgGetNumOfSensor.restype = ctypes.c_int
    num_sensor = lib.wgGetNumOfSensor()
    print("glove connected")
    print(f'num_sensor: {num_sensor}')
    sn_str = b'aaaaaaaaa'
    snfunc(sn_str)
    print(f"该手套序列号为：{sn_str.decode('utf-8')}")
# print(regdict)

#lib.wgSetCalibMode(ctypes.c_int(0))#自动标定

lib.wgResetCalib()
lib.wgGetCalibRange.restype = ctypes.c_float
for i in range(18):
    angle_range[i] = lib.wgGetCalibRange(ctypes.c_int(i))
    # print(f"angle_range[{i+1}]: {angle_range[i]}")

# for key,value in joints.items():
#         print (f"{key}  ",end="")
# print('\n')

################################################ inspire hand init ################################################
print("打开灵巧手串口")
ser = openSerial('COM6', 115200)
forceClb(ser,1)
inspire_ready = False

################################################# main loop #################################################
while True:
    if msvcrt.kbhit():
        key=msvcrt.getch()
        #print(type(key))
        keystr=str(key, encoding="utf-8")
        if (keystr=='q'):
                break
        if (keystr=='r'):
            lib.wgResetCalib()
            for i in range(18):
                angle_range[i] = lib.wgGetCalibRange(ctypes.c_int(i))
        if (keystr=='c'):
            inspire_ready = True

    lib.wgGetAngle.restype = ctypes.c_uint
    timestamp = lib.wgGetAngle(angle)
    write_angle = init_pos

    write_angle[0] = int(1000-(angle[joints["Pin_MCP"] - 1]/angle_range[joints["Pin_MCP"] - 1] * 1000))
    write_angle[1] = int(1000-(angle[joints["Rin_MCP"] - 1]/angle_range[joints["Rin_MCP"] - 1] * 1000))
    write_angle[2] = int(1000-(angle[joints["Mid_MCP"] - 1]/angle_range[joints["Mid_MCP"] - 1] * 1000))
    write_angle[3] = int(1000-(angle[joints["Ind_MCP"] - 1]/angle_range[joints["Ind_MCP"] - 1] * 1000))
    write_angle[4] = int(1000-(angle[joints["Thu_MCP"] - 1]/angle_range[joints["Thu_MCP"] - 1] * 1000))
    write_angle[5] = int(1000-(angle[joints["Thu_CMC"] - 1]/angle_range[joints["Thu_CMC"] - 1] * 1000))
    print(f"Pin: {write_angle[0]}   Rin: {write_angle[1]}   Mid: {write_angle[2]}   Ind: {write_angle[3]}  Thu: {write_angle[4]}  Thu_CMC: {write_angle[5]}")

    if timestamp > 0 and inspire_ready:
        write_data_6(ser,1,'angleSet',write_angle)

