#!/usr/bin/env python3

import rospy
import os
import threading, time
import sys
import time
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

    pub_stop.publish(stop_mode = STOP_TYPE_QUICK)
    return 0

a0509 = CDsrRobot(ROBOT_ID, ROBOT_MODEL)

FlagJog = -1
flagAxes = False
flagPad = False
xyAxes = False
joyJogVel = 0.0
joyJogAxis = 0
Vel = 20
moveReference = MOVE_REFERENCE_BASE


def joystick(msg):

    homePosj0 = [0, 0, 0, 0, 0, 0]
    homePosj1 = [0, 0, 90, 0, 90, 0]

    global FlagJog
    global flagAxes
    global flagPad
    global xyAxes
    global joyJogVel
    global joyJogAxis
    global Vel
    global moveReference

    # 回到两个预定位姿
    if msg.buttons[8] == 1:
        a0509.movej(homePosj0, vel=30, acc=30)
    elif msg.buttons[7] == 1:
        a0509.movej(homePosj1, 30, 30)
    
    # 切换运动参考坐标系
    if msg.buttons[6] == 1:
        if moveReference == MOVE_REFERENCE_TOOL:
            moveReference = MOVE_REFERENCE_BASE
        else:
            moveReference = MOVE_REFERENCE_TOOL

    # 切换运动速度
    if msg.buttons[3] == 1:
        if Vel < 60:
            Vel += 20
        else:
            Vel = 20

    # 按住A健才可以使用十字键和左右肩健移动机械臂
    if msg.buttons[0] == 1 and msg.buttons[1] == 0 and (msg.axes[6]!=0 or msg.axes[7]!=0 or msg.buttons[4]!=0 or msg.buttons[5]!=0) :
        flagPad = True
    else:
        flagPad = False

    # 按住B健才可以使用左摇杆和左右扳机移动机械臂
    if msg.buttons[0] == 0 and msg.buttons[1] == 1 and (msg.axes[0]!=0 or msg.axes[1]!=0 or msg.axes[2]!=1.0 or msg.axes[5]!=1.0):
        flagAxes = True
    else:
        flagAxes = False

    if msg.axes[1] != 0 or msg.axes[0] or 0:
        if abs(msg.axes[1]) > abs(msg.axes[0]):
            xyAxes = False
        else:
            xyAxes = True

    if FlagJog == -1 and flagPad and not flagAxes:
        if msg.axes[6] == 1:
            FlagJog = JOG_AXIS_TASK_Y
            joyJogVel = -Vel
        if msg.axes[6] == -1:
            FlagJog = JOG_AXIS_TASK_Y
            joyJogVel = Vel
        if msg.axes[7] == 1:
            FlagJog = JOG_AXIS_TASK_X
            joyJogVel = Vel
        if msg.axes[7] == -1:
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

    elif FlagJog == -1 and not flagPad and flagAxes:
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

    else:
        if not flagPad and not flagAxes:
            rospy.loginfo(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}]||||||jog stop")
            a0509.jog(FlagJog, moveReference, 0)
            FlagJog = -1



if __name__ == "__main__":
    rospy.init_node('logi_joy_py')
    rospy.on_shutdown(shutdown)

    pub_stop = rospy.Publisher('/' + ROBOT_ID + ROBOT_MODEL + '/stop', RobotStop, queue_size=Vel)
    sub_joy = rospy.Subscriber("joy", Joy, joystick)
    while not rospy.is_shutdown():
        pass
    