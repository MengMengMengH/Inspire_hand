from ctypes import *
import time
import ctypes
import sys, os
from pynput.keyboard import Controller,Key,Listener
from pynput import keyboard

import numpy as np
import msvcrt


from ins_read import *      # inspire_hand


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
lib = CDLL("./wiseglove/WISEGLOVEU3D.dll")

g_pGlove = ctypes.c_bool
num_sensor = ctypes.c_int

angle = (c_float*18)()
sensor = (c_ushort*18)()
angle_range = (c_float*18)()

timestamp = ctypes.c_uint


################################################ data glove init ################################################

def init_glove():
    lib.wgInit.restype = ctypes.c_bool
    g_pGlove=lib.wgInit(1)  # 0=左手,1=右手,-1=不分左右 
    if not g_pGlove:
        print("No Glove detected.")
        lib.wgClose()
        sys.exit(1)


    lib.wgGetNumOfSensor.restype = ctypes.c_int
    num_sensor = lib.wgGetNumOfSensor()

    print("glove connected")
    print(f'num_sensor: {num_sensor}')

    sn_str = b'aaaaaaaaa'
    snfunc = lib.wgGetSn
    snfunc.restype = ctypes.c_bool
    snfunc.argtypes = [ctypes.POINTER(ctypes.c_char)]
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
    return num_sensor

################################################ inspire hand init ################################################
def init_inspire():
    print("打开灵巧手串口")
    ser = openSerial('COM6', 115200)
    forceClb(ser,1)
    inspire_ready = False
    return ser, inspire_ready



############################################### mapping data to inspire ###############################################
def mapping_ang(mcp="Thu_MCP", ip="Thu_IP", co_thu=[0.5, 0.5]):
    """
    计算角度。

    参数：
    - mcp: 拇指 MCP 关节名称，默认 "Thu_MCP"
    - ip: 拇指 IP 关节名称，默认 "Thu_IP"
    - co_thu: 权重系数，默认为 [0.5, 0.5]

    返回：
    - thu_ang: 计算得到的拇指角度
    """
    # 获取拇指 MCP 和 IP 的关节角度和角度范围
    mcp_angle = angle[joints[mcp] - 1]
    ip_angle = angle[joints[ip]-1]
    mcp_range = angle_range[joints[mcp] - 1]
    ip_range = angle_range[joints[ip] - 1]

    # 计算 ang
    ang = 1000 - (co_thu[0] * mcp_angle / mcp_range + co_thu[1] * ip_angle / ip_range) * 1000

    return ang

################################################# main loop #################################################
def teleop(ser, inspire_ready):
    while True:
        if msvcrt.kbhit():
            key=msvcrt.getch()
            #print(type(key))
            keystr=str(key, encoding="utf-8")

            if (keystr=='q'):
                    break
            elif (keystr=='r'):
                lib.wgResetCalib()
                for i in range(18):
                    angle_range[i] = lib.wgGetCalibRange(ctypes.c_int(i))
            elif (keystr=='c'):
                inspire_ready = True

        lib.wgGetAngle.restype = ctypes.c_uint
        timestamp = lib.wgGetAngle(angle)
        write_angle = init_pos

        thu_ang = mapping_ang(mcp = "Thu_MCP", ip = "Thu_IP", co_thu = [0.5, 0.5])
        ind_ang = mapping_ang(mcp = "Ind_MCP", ip = "Ind_PIP", co_thu = [0.5, 0.5])
        mid_ang = mapping_ang(mcp = "Mid_MCP", ip = "Mid_PIP", co_thu = [0.5, 0.5])
        rin_ang = mapping_ang(mcp = "Rin_MCP", ip = "Rin_PIP", co_thu = [0.5, 0.5])
        pin_ang = mapping_ang(mcp = "Pin_MCP", ip = "Pin_DIP", co_thu = [0.5, 0.5])
        thu_cmc_ang = 1000 - (angle[joints["Thu_CMC"] - 1]/angle_range[joints["Thu_CMC"] - 1] * 1000)
        # print(f"ind_ang:{ind_ang}")

        if timestamp > 0:
            write_angle[0] = int(pin_ang)
            write_angle[1] = int(rin_ang)
            write_angle[2] = int(mid_ang)
            write_angle[3] = int(ind_ang)
            write_angle[4] = int(thu_ang)
            write_angle[5] = int(thu_cmc_ang)
        print(f"Pin: {write_angle[0]}   Rin: {write_angle[1]}   Mid: {write_angle[2]}   Ind: {write_angle[3]}  Thu: {write_angle[4]}  Thu_CMC: {write_angle[5]}")

        if inspire_ready:
            write_data_6(ser,1,'angleSet',write_angle)


if __name__ == '__main__':

    try:

        num_sensor = init_glove()
        ser, inspire_ready = init_inspire()
        teleop(ser, inspire_ready)

    finally:

        lib.wgClose()
        ser.close()
        print("Program ended.")
        sys.exit(0)