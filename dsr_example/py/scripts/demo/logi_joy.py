#!/usr/bin/env python3

import rospy
import os
import threading, time
import sys
import time
import tkinter as tk
from sensor_msgs.msg import Joy
from datetime import datetime
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../common/imp")))

ROBOT_ID = "dsr01"
ROBOT_MODEL = "a0509"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

def shutdown():
    print()
    print("SHUTDOWN TIME!!!")
    print()
    print("SHUTDOWN TIME!!!")
    print()
    print("SHUTDOWN TIME!!!")

    window.destroy()

    pub_stop.publish(stop_mode = STOP_TYPE_QUICK)
    return 0

a0509 = CDsrRobot(ROBOT_ID, ROBOT_MODEL)

FlagJog = -1
flagAxes = False
flagPad = False
flagSingle = False
xyAxes = False
joyJogVel = 0.0
Vel = 20
Axis = JOG_AXIS_JOINT_1
moveReference = MOVE_REFERENCE_TOOL

window = tk.Tk()
window.title("JoyStick Control")

labelHint = tk.Label(window, text="Use LS to change velocity. \nUse RS to switch axis. \nUse Y to switch ReferenceCoor. ")
labelHint.pack(pady=10)

labelVel = tk.Label(window, text="Vel: 20%")
labelVel.pack(pady=20)

labelRef = tk.Label(window, text="Ref: MOVE_REFERENCE_TOOL")
labelRef.pack(pady=20)

labelAxis = tk.Label(window, text="Axis: 1")
labelAxis.pack(pady=20)

def joystick(msg):

    homePosj0 = [0, 0, 0, 0, 0, 0]
    homePosj1 = [0, 0, 90, 0, 0, 0]

    global FlagJog
    global flagAxes
    global flagPad
    global flagSingle
    global xyAxes
    global joyJogVel
    global Vel
    global Axis
    global moveReference

    # 回到两个预定位姿
    if msg.buttons[8] == 1:
        a0509.movej(homePosj0, vel=20, acc=30)
    elif msg.buttons[7] == 1:
        a0509.movej(homePosj1, vel=20, acc=30)

    # 使用LS键切换运动速度
    if msg.buttons[9] == 1:
        if Vel < 60:
            Vel += 20
        else:
            Vel = 20
        labelVel.config(text="Vel: " + str(Vel) + "%")

    # 使用RS键切换轴控制的对应轴
    if msg.buttons[10] == 1: 
        if Axis < 5: 
            Axis += 1
        else: 
            Axis = 0
        labelAxis.config(text="Axis: " + str(Axis+1))
    
    # 使用Y键切换运动参考坐标系
    if msg.buttons[3] == 1:
        if moveReference == MOVE_REFERENCE_TOOL:
            moveReference = MOVE_REFERENCE_BASE
            labelRef.config(text="Ref: MOVE_REFERENCE_BASE")
        elif moveReference == MOVE_REFERENCE_BASE:
            moveReference = MOVE_REFERENCE_TOOL
            labelRef.config(text="Ref: MOVE_REFERENCE_TOOL")

    # 按住A健才可以使用十字键和左右肩健移动机械臂
    if msg.buttons[0] == 1 and msg.buttons[1] == 0 and msg.buttons[2] == 0 and (msg.axes[6]!=0 or msg.axes[7]!=0 or msg.buttons[4]!=0 or msg.buttons[5]!=0) :
        flagPad = True
    else:
        flagPad = False

    # 按住B健才可以使用左摇杆和左右扳机移动机械臂
    if msg.buttons[0] == 0 and msg.buttons[1] == 1 and msg.buttons[2] == 0 and (msg.axes[0]!=0 or msg.axes[1]!=0 or msg.axes[2]!=1.0 or msg.axes[5]!=1.0):
        flagAxes = True
    else:
        flagAxes = False

    if msg.axes[1] != 0 or msg.axes[0] or 0:
        if abs(msg.axes[1]) > abs(msg.axes[0]):
            xyAxes = False
        else:
            xyAxes = True

    # 按住X键才可以使用十字键转动单个轴
    if msg.buttons[0] == 0 and msg.buttons[1] == 0 and msg.buttons[2] == 1 and msg.axes[6]!=0: 
        flagSingle = True
    else: 
        flagSingle = False

    if FlagJog == -1 and flagPad and not flagAxes and not flagSingle:
        if msg.axes[7] == 1:
            FlagJog = JOG_AXIS_TASK_Y
            joyJogVel = -Vel
        if msg.axes[7] == -1:
            FlagJog = JOG_AXIS_TASK_Y
            joyJogVel = Vel
        if msg.axes[6] == 1:
            FlagJog = JOG_AXIS_TASK_X
            joyJogVel = Vel
        if msg.axes[6] == -1:
            FlagJog = JOG_AXIS_TASK_X
            joyJogVel = -Vel
        if msg.buttons[4] == 1:
            FlagJog = JOG_AXIS_TASK_Z
            joyJogVel = -Vel
        if msg.buttons[5] == 1:
            FlagJog = JOG_AXIS_TASK_Z
            joyJogVel = Vel

        print("Now Jogging: Vel = ", joyJogVel)
        print("Now Jogging: Ref = ", moveReference)
        a0509.jog(FlagJog, moveReference, joyJogVel)

    elif FlagJog == -1 and not flagPad and flagAxes and not flagSingle:
        if msg.axes[2] < 1.0:
            FlagJog = JOG_AXIS_TASK_RZ
            joyJogVel = -Vel
        if msg.axes[5] < 1.0:
            FlagJog = JOG_AXIS_TASK_RZ
            joyJogVel = Vel
        if msg.axes[1] > 0 and xyAxes == 0:
            FlagJog = JOG_AXIS_TASK_RX
            joyJogVel = Vel
        if msg.axes[1] < 0 and xyAxes == 0:
            FlagJog = JOG_AXIS_TASK_RX
            joyJogVel = -Vel
        if msg.axes[0] > 0 and xyAxes == 1:
            FlagJog = JOG_AXIS_TASK_RY
            joyJogVel = -Vel
        if msg.axes[0] < 0 and xyAxes == 1:
            FlagJog = JOG_AXIS_TASK_RY
            joyJogVel = Vel
        
        print("Now Jogging: Vel = ", joyJogVel)
        print("Now Jogging: Ref = ", moveReference)
        a0509.jog(FlagJog, moveReference, joyJogVel)

    elif FlagJog == -1 and flagSingle and not flagAxes and not flagPad: 
        if msg.axes[6] > 0: 
            FlagJog = Axis
            joyJogVel = Vel
        if msg.axes[6] < 0: 
            FlagJog = Axis
            joyJogVel = -Vel

        print("Now Jogging: Vel = ", joyJogVel)
        print("Now Jogging: Axis = ", Axis+1)
        print("Now Jogging: Ref = ", moveReference)
        a0509.jog(FlagJog, moveReference, joyJogVel)

    else:
        if not flagPad and not flagAxes:
            rospy.loginfo(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}]||||||jog stop")
            a0509.jog(FlagJog, moveReference, 0)
            FlagJog = -1

if __name__ == "__main__":
    rospy.init_node('logi_joy_py')
    rospy.on_shutdown(shutdown)

    pub_stop = rospy.Publisher('/' + ROBOT_ID + ROBOT_MODEL + '/stop', RobotStop, queue_size=10)
    sub_joy = rospy.Subscriber("joy", Joy, joystick)

    window.mainloop()

    while not rospy.is_shutdown():
        pass
    
