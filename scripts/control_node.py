#!/usr/bin/env python3
from ast import If
import sys
# check pythone version
if sys.version_info < (3,0):
    print("Sorry, requires Python 3.x, not Python 2.x")
    sys.exit(1)

# TF fix Verion 2
try:
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
except:
    sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
    sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
    sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
    sys.path.insert(0,'/opt/ros/melodic/lib/python2.7/dist-packages')


import rospy
import math
import message_filters # To Achieve Multiple subscriber
from std_msgs.msg import String
import threading
from multiprocessing import Process
import tkinter as tk
import tkinter.font as tkfont
import time

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

from jsk_recognition_msgs.msg import BoolStamped    # for navigation node message filters use

class drone_control_node(object):
    
    def __init__(self):
        # Tells rospy the name of the node.
        # Anonymous = True makes sure the node has a unique name. Random
        # numbers are added to the end of the name.
        rospy.init_node("drone_control", anonymous=True, disable_signals=True)

        # ROS loop rate
        # Rate
        self.loop_rate = rospy.Rate(40)
        
        # Node is publishing to the topic
        # e.g. self.vesc1_pub = rospy.Publisher(vesc1_ns + '/commands/motor/speed', Float64, queue_size=10)
        self.dron_position_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.dron_control_mode_pub = rospy.Publisher("/drone/current/control_mode", String, queue_size=1)
        self.dron_nagvation_pose_pub = rospy.Publisher("/drone/nagvation/pos", PoseStamped, queue_size=1)
        # un doc change
        self.automode_pub = rospy.Publisher("/auto_mode/status", BoolStamped, queue_size=1)
        
        # Node is subscribing to the topic
        # Subscribe to drone state
        self.local_pos_sub = message_filters.Subscriber("/mavros/local_position/pose", PoseStamped)
        self.local_pos_sub.registerCallback(self.callback_local_position)
        self.target_pos_sub = message_filters.Subscriber("/drone/input_postion/pose", PoseStamped)
        self.target_pos_sub.registerCallback(self.callback_target_position)
        self.px4_state_sub = message_filters.Subscriber("/mavros/state", State)
        self.px4_state_sub.registerCallback(self.callback_px4_state)
        self.base_link="base_link"
        
        #setup variable
        self.mode="manual"
        
        # Manual Mode false safe
        self.dronePosSafe = False
        #target POS by GUI manual input
        self.dronePosX =0.0
        self.dronePosY =0.0
        self.dronePosZ =0.0
        #-For rotate only z should be change
        self.droneRotX =0.0
        self.droneRotY =0.0
        self.droneRotZ =0.0


        #local POS return by PX4
        self.droneLocalPosX =0.0
        self.droneLocalPosY =0.0
        self.droneLocalPosZ =0.0
        self.droneLocalRotX =0.0
        self.droneLocalRotY =0.0
        self.droneLocalRotZ =0.0
        self.droneLocalQuatX =0.0
        self.droneLocalQuatY =0.0
        self.droneLocalQuatZ =0.0
        self.droneLocalQuatW =1.0

        #safe local
        #local POS return by PX4
        self.droneSafeLocalPosX =0.0
        self.droneSafeLocalPosY =0.0
        self.droneSafeLocalPosZ =0.0
        self.droneSafeLocalQuatX =0.0
        self.droneSafeLocalQuatY =0.0
        self.droneSafeLocalQuatZ =0.0
        self.droneSafeLocalQuatW =1.0


        # Auto mode false safe
        self.droneTargetPosSafe = False
        # target POS by GUI manual input
        self.droneTargetPosX =0.0
        self.droneTargetPosY =0.0
        self.droneTargetPosZ =0.0
        #-For rotate only yaw should be change
        self.droneTargetQuatX =0.0
        self.droneTargetQuatY =0.0
        self.droneTargetQuatZ =0.0
        self.droneTargetQuatW =0.0

        # Auto mode false safe
        self.droneNavPosSafe = False
        #target POS by Navgoation(input)
        self.droneNavtime =rospy.Time.now()
        self.droneNavPosX =0.0
        self.droneNavPosY =0.0
        self.droneNavPosZ =0.0
        #-For rotate only yaw should be change
        self.droneNavQuatX =0.0
        self.droneNavQuatY =0.0
        self.droneNavQuatZ =0.0
        self.droneNavQuatW =0.0

        self.root = tk.Tk()
        #Navgoation post
        self.droneNavPosXInfo = tk.StringVar(self.root, value="0.0")
        self.droneNavPosYInfo = tk.StringVar(self.root, value="0.0")
        self.droneNavPosZInfo = tk.StringVar(self.root, value="0.0")
        self.droneNavRotXInfo = tk.StringVar(self.root, value="0.0")
        self.droneNavRotYInfo = tk.StringVar(self.root, value="0.0")
        self.droneNavRotZInfo = tk.StringVar(self.root, value="0.0")

        #Local Pos
        self.droneLocPosXInfo = tk.StringVar(self.root, value="0.0")
        self.droneLocPosYInfo = tk.StringVar(self.root, value="0.0")
        self.droneLocPosZInfo = tk.StringVar(self.root, value="0.0")
        self.droneLocRotXInfo = tk.StringVar(self.root, value="0.0")
        self.droneLocRotYInfo = tk.StringVar(self.root, value="0.0")
        self.droneLocRotZInfo = tk.StringVar(self.root, value="0.0")

        #px4 state
        self.isPX4_armed= False
        self.isPX4_connected= False
        self.px4_mode= ""


    #callback_local_position
    def callback_local_position(self, msg):
        #rospy.loginfo("Drone Control Node: local pos call back")
        self.droneLocalPosX =msg.pose.position.x
        self.droneLocalPosY =msg.pose.position.y
        self.droneLocalPosZ =msg.pose.position.z
        orientation_quaternion = msg.pose.orientation
        orientation_quaternion_list = [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w]
        #(roll, pitch, yaw)
        (self.droneLocalRotX, self.droneLocalRotY, self.droneLocalRotZ) = euler_from_quaternion (orientation_quaternion_list)
        self.droneLocalQuatX = msg.pose.orientation.x
        self.droneLocalQuatY = msg.pose.orientation.y
        self.droneLocalQuatZ = msg.pose.orientation.z
        self.droneLocalQuatW = msg.pose.orientation.w
        #rospy.loginfo("Drone Control Node: local pos X:%f,Y:%f,Z:%f",msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
        #rospy.loginfo("Drone Control Node: local pos X:%f,Y:%f,Z:%f",self.droneLocalPosX,self.droneLocalPosY,self.droneLocalPosZ)

    #callback_target_position(input)
    def callback_target_position(self, msg):
        #target POS by Navgoation
        self.droneNavPosX =msg.pose.position.x
        self.droneNavPosY =msg.pose.position.y
        self.droneNavPosZ =msg.pose.position.z
        self.droneNavtime =msg.header.stamp
        #-For rotate only yaw should be change
        #rospy.loginfo(msg.pose.orientation)
        self.droneNavQuatX = msg.pose.orientation.x
        self.droneNavQuatY = msg.pose.orientation.y
        self.droneNavQuatZ = msg.pose.orientation.z
        self.droneNavQuatW = msg.pose.orientation.w
        self.droneNavPosSafe =True

    #callback_px4_postion(input)
    def callback_px4_state(self, msg):
        self.isPX4_armed= msg.armed
        self.isPX4_connected= msg.connected
        self.px4_mode= msg.mode


    #valid is number(float)
    def validateIsFloat(self, action, index, value_if_allowed, prior_value, text, validation_type, trigger_type, widget_name):
        if value_if_allowed:
            try:
                float(value_if_allowed)
                return True
            except ValueError:
                return False
        else:
            return False

    #Control Node Mode
    def changeModeSafety(self):
        self.dronePosSafe = False
        self.droneNavPosSafe = False
        self.droneTargetPosSafe = False
        self.droneSafeLocalPosX =self.droneLocalPosX
        self.droneSafeLocalPosY =self.droneLocalPosY
        self.droneSafeLocalPosZ =self.droneLocalPosZ
        (self.droneSafeLocalQuatX, self.droneSafeLocalQuatY, self.droneSafeLocalQuatZ, self.droneSafeLocalQuatW) = quaternion_from_euler(0, 0, self.droneLocalRotZ)

        
    #-Vision
    def autoMode(self):
        self.changeModeSafety()
        self.mode="auto"
        #add this to make hei happy :)
        self.setDronPos()
        rospy.loginfo("Drone Control Node: Setting Control to Auto Navgoation....")


    #-Pos
    def visionPosMode(self):
        self.changeModeSafety()
        self.mode="visionPos"
        rospy.loginfo("Drone Control Node: Setting Control to Vision Position....")

    #-manual
    #--GUI set px4 target pos
    def manualMode(self):
        self.changeModeSafety()
        self.mode="manual"
        rospy.loginfo("Drone Control Node: Setting Control to Manual....")


    #PX4 Mode
    #-arm PX4
    def setPx4ArmMode(self):
        rospy.loginfo("Drone Control Node: Trying PX4 Mode to Arm....")
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            rospy.logwarn("[Drone Control Node] Aming service failed: %s"%e)
    def px4ArmMode(self):
        rospy.loginfo("Drone Control Node: Starting PX4 Mode to Arm")
        self.armDaemon = threading.Thread(target = self.setPx4ArmMode(),daemon = True)
        self.armDaemon.start()

    #-px4 setmode
    def px4SetMode(self, setToMode):
        rospy.loginfo("Drone Control Node: Trying to set PX4 Mode to %s",setToMode)
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode= setToMode)
        except rospy.ServiceException as e:
            rospy.logwarn("[Drone Control Node] Aming service failed: %s"%e)

    #-px4 to Position Mode
    def px4PosMode(self):
        #later do
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to Position Mode....")
        self.modeDaemon = threading.Thread(target = self.px4SetMode('POSCTL'),daemon = True)
        self.modeDaemon.start()

    #-px4 to OffBoard Mode
    def px4OffBoardMode(self):
        #later do
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to Offboard Mode")
        self.modeDaemon = threading.Thread(target = self.px4SetMode('OFFBOARD'),daemon = True)
        self.modeDaemon.start()

    def px4LandMode(self):
        #later do
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to Offboard Mode")
        self.modeDaemon = threading.Thread(target = self.px4SetMode('AUTO.LAND'),daemon = True)
        self.modeDaemon.start()

    def px4RTLMode(self):
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to AUTO.RTL(Return to Staring) Mode")
        self.modeDaemon = threading.Thread(target = self.px4SetMode('AUTO.RTL'),daemon = True)
        self.modeDaemon.start()

    def setDronPosXYZ(self,X=0.0,Y=0.0,Z=0.0,Zr=0.0):
        #target POS by GUI manual input
        self.droneTargetPosX =X
        self.droneTargetPosY =Y
        self.droneTargetPosZ =Z
        #-For rotate only yaw should be change
        (self.droneTargetQuatX, self.droneTargetQuatY, self.droneTargetQuatZ, self.droneTargetQuatW) = quaternion_from_euler(0, 0, math.radians(Zr))
        self.droneTargetPosSafe = True
        self.dronePosSafe = True

    def setDronPos(self):
        self.setDronPosXYZ(float(self.dronePosXInp.get()), float(self.dronePosYInp.get()), float(self.dronePosZInp.get()),float(self.droneRotZInp.get()))

    #Drone movement
    #-Horizontal Movement    
    def droneForward(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Forward")
        #modeDaemon = threading.Thread(target = self.px4SetMode('OFFBOARD'),daemon = True
    
    def droneLeft(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Left")
    
    def droneStop(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Stop")

    def droneRight(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Right")
    
    def droneBackward(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Backward")

    #-Vertical Movement
    def droneUp(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Backward")

    def takeLocal(self):
        self.dronePosXInp.set(round(self.droneLocalPosX,3))
        self.dronePosYInp.set(round(self.droneLocalPosY,3))
        self.dronePosZInp.set(round(self.droneLocalPosZ,3))
        self.droneRotZInp.set(round(math.degrees(self.droneLocalRotZ),3))
    
    def stopmanual(self):
        self.manualMode()
        self.setDronPosXYZ(self.droneLocalPosX,self.droneLocalPosY,self.droneLocalPosZ,self.droneLocalRotZ)


    #GUI Body
    def gui(self):
        rospy.loginfo("Drone Control Node: Setting PX4 Mode up GUI now")
        #self.root.destroy()
        #self.root = tk.Tk()
        self.root.title("Control Node GUI: Onboard")
        #self.root.configure(bg='grey')

        #title 
        titlef = tk.Frame(self.root)
        menubar = tk.Menu(self.root)
        menuList = tk.Menu(menubar, tearoff=0)
        menuList.add_command(label="Exit", command=lambda:self.root.destroy())
        menubar.add_cascade(label="Menu", menu=menuList)
        self.root.config(menu=menubar)

        #title style
        title_font = tkfont.Font(family='Helvetica', size=24, weight="bold", slant="italic")

        #title body
        title = tk.Label(titlef, text="Control Node: GUI", font=title_font, anchor="e" )
        title.grid(row=0, column=1,sticky="")
        titlef.grid(row=0, column=0, columnspan=5,sticky="")

        #Select Control Mode
        target = tk.LabelFrame(self.root, text="Target Control Mode",width=200)
        target.grid(row=1, column=0, columnspan=3,sticky="W")

        self.autoModeBut = tk.Button(target, text="Auto", width=10,bd=2, cursor="exchange", command = lambda: self.autoMode())
        self.autoModeBut.grid(row=1, column=1, columnspan=1) 

        self.semiHardModeBut = tk.Button(target, text="Remote", width=10,bd=2, cursor="exchange", command = lambda: self.visionPosMode())
        self.semiHardModeBut.grid(row=1, column=2, columnspan=1)

        self.manualModeBut = tk.Button(target, text="Manual", width=10,bd=2, cursor="exchange", command = lambda: self.manualMode())
        self.manualModeBut.grid(row=1, column=0, columnspan=1)
        
        #Select Px4 Flight Mode
        self.px4Mode = tk.LabelFrame(self.root, text="PX4 Flight Mode",width=200)
        self.px4Mode.grid(row=2, column=0, columnspan=5,sticky="W")

        self.px4ArmBut = tk.Button(self.px4Mode, text="Arm", width=10,bd=2, cursor="exchange", command = lambda: self.px4ArmMode())
        self.px4ArmBut.grid(row=1, column=0, columnspan=1) 

        self.px4PosBut = tk.Button(self.px4Mode, text="Position Mode", width=10,bd=2, cursor="exchange", command = lambda: self.px4PosMode())
        self.px4PosBut.grid(row=1, column=2, columnspan=1) 

        self.px4OffBoardBut = tk.Button(self.px4Mode, text="Offboard Mode", width=10,bd=2, cursor="exchange", command = lambda: self.px4OffBoardMode())
        self.px4OffBoardBut.grid(row=1, column=1, columnspan=1) 

        self.px4LandBut = tk.Button(self.px4Mode, text="Auto Land", width=10,bd=2, cursor="exchange", command = lambda: self.px4LandMode())
        self.px4LandBut.grid(row=1, column=3, columnspan=1) 

        self.px4RTLBut = tk.Button(self.px4Mode, text="Return To Start", width=15,bd=2, cursor="exchange", command = lambda: self.px4RTLMode())
        self.px4RTLBut.grid(row=1, column=4, columnspan=1) 

        #Drone Movement
        droneMovement = tk.LabelFrame(self.root, text="Drone Movement",width=200)
        droneMovement.grid(row=3, column=0, columnspan=10,sticky="W")
        
        #Horizontal Movement
        droneHorizontalMovement = tk.LabelFrame(droneMovement, text="Horizontal Movement",width=200)
        droneHorizontalMovement.grid(row=0, column=0, columnspan=3,rowspan=3, sticky="N")

        droneForwardBut = tk.Button(droneHorizontalMovement, text="Forward", width=10,bd=2, cursor="exchange", command = lambda: self.droneForward())
        droneForwardBut.grid(row=0, column=1, columnspan=1) 

        droneLeftBut = tk.Button(droneHorizontalMovement, text="Left", width=10,bd=2, cursor="exchange", command = lambda: self.droneLeft())
        droneLeftBut.grid(row=1, column=0, columnspan=1) 

        droneStopBut = tk.Button(droneHorizontalMovement, text="Hold", width=10,bd=2, cursor="exchange", command = lambda: self.droneStop())
        droneStopBut.grid(row=1, column=1, columnspan=1) 

        droneRightBut = tk.Button(droneHorizontalMovement, text="Right", width=10,bd=2, cursor="exchange", command = lambda: self.droneRight())
        droneRightBut.grid(row=1, column=2, columnspan=1) 

        droneBackwardBut = tk.Button(droneHorizontalMovement, text="Backward", width=10,bd=2, cursor="exchange", command = lambda: self.droneBackward())
        droneBackwardBut.grid(row=2, column=1, columnspan=1) 

        #Vertical Movement
        droneVerticalMovement = tk.LabelFrame(droneMovement, text="Vertical Movement",width=200)
        droneVerticalMovement.grid(row=0, column=3, columnspan=1,rowspan=3,sticky="N")

        droneUpBut = tk.Button(droneVerticalMovement, text="Up", width=10,bd=2, cursor="exchange", command = lambda: self.droneUp())
        droneUpBut.grid(row=0, column=0, columnspan=1) 

        droneStop2But = tk.Button(droneVerticalMovement, text="Hold", width=10,bd=2, cursor="exchange", command = lambda: self.droneStop())
        droneStop2But.grid(row=1, column=0, columnspan=1)

        droneDownBut = tk.Button(droneVerticalMovement, text="Down", width=10,bd=2, cursor="exchange", command = lambda: self.droneDown())
        droneDownBut.grid(row=2, column=0, columnspan=1) 

        #Rotational Movement
        droneRotationalMovement = tk.LabelFrame(droneMovement, text="Rotational Movement",width=200)
        droneRotationalMovement.grid(row=0, column=4, columnspan=2,rowspan=3,sticky="N")

        droneRotateLeftBut = tk.Button(droneRotationalMovement, text="Rotate Left", width=10,bd=2, cursor="exchange", command = lambda: self.droneRotateLeft())
        droneRotateLeftBut.grid(row=0, column=0, columnspan=1) 

        droneRotateRightBut = tk.Button(droneRotationalMovement, text="Rotate Right", width=10,bd=2, cursor="exchange", command = lambda: self.droneRotateRight())
        droneRotateRightBut.grid(row=0, column=1, columnspan=1)

        #Drone Movement by Pos
        dronePosMovement = tk.LabelFrame(self.root, text="Drone Movement(By Position)",width=200)
        dronePosMovement.grid(row=6, column=0, columnspan=10,rowspan=2,sticky="W")
        
        #position X
        dronePosXLab = tk.Label(dronePosMovement, text="X:")
        dronePosXLab.grid(row=0, column=0, columnspan=1,sticky="W")

        vcmdPosX = (self.root.register(self.validateIsFloat),'%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        self.dronePosXInp = tk.StringVar(self.root, value="0.0") 
        dronePosXEnt = tk.Entry(dronePosMovement, validate = 'key', validatecommand = vcmdPosX, textvariable = self.dronePosXInp)
        #dronePosXEnt = tk.Entry(dronePosMovement, textvariable = self.dronePosX)
        dronePosXEnt.grid(row=0,column=1)
        
        #position Y
        dronePosYLab = tk.Label(dronePosMovement, text="Y:")
        dronePosYLab.grid(row=0, column=2, columnspan=1,sticky="W")

        vcmdPosY = (self.root.register(self.validateIsFloat),'%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        self.dronePosYInp = tk.StringVar(self.root, value="0.0")
        dronePosYEnt = tk.Entry(dronePosMovement, validate = 'key', validatecommand = vcmdPosY, textvariable = self.dronePosYInp)
        #dronePosYEnt = tk.Entry(dronePosMovement, textvariable = self.dronePosY)
        dronePosYEnt.grid(row=0,column=3)


        #position Z
        dronePosZLab = tk.Label(dronePosMovement, text="Z:")
        dronePosZLab.grid(row=0, column=4, columnspan=1,sticky="W")

        vcmdPosZ = (self.root.register(self.validateIsFloat),'%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        self.dronePosZInp = tk.StringVar(self.root, value="0.0")
        dronePosZEnt = tk.Entry(dronePosMovement, validate = 'key', validatecommand = vcmdPosZ, textvariable = self.dronePosZInp)
        dronePosZEnt.grid(row=0,column=5)
        
        #Rotation Z
        droneRotZLab = tk.Label(dronePosMovement, text="Rotation Z:")
        droneRotZLab.grid(row=0, column=6, columnspan=1,sticky="W")

        vcmdRotZ = (self.root.register(self.validateIsFloat),'%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        self.droneRotZInp = tk.StringVar(self.root, value="0.0")
        droneRotZEnt = tk.Entry(dronePosMovement, validate = 'key', validatecommand = vcmdRotZ, textvariable = self.droneRotZInp)
        droneRotZEnt.grid(row=0,column=7)

        #Set pos
        autoModeBut = tk.Button(dronePosMovement, text="Set", width=10, bd=2, cursor="exchange", command = lambda: self.setDronPos())
        autoModeBut.grid(row=1, column=0, columnspan=2) 
        #reset manual pos
        autoModeBut = tk.Button(dronePosMovement, text="Take Local Pos", width=10, bd=2, cursor="exchange", command = lambda: self.takeLocal())
        autoModeBut.grid(row=1, column=2, columnspan=2) 
        #stop manual pos
        autoModeBut = tk.Button(dronePosMovement, text="Stop", width=10, bd=2, cursor="exchange", command = lambda: self.stopmanual())
        autoModeBut.grid(row=1, column=4, columnspan=2) 



        #Display Local Pos
        droneLocPos = tk.LabelFrame(self.root, text="Current Position:",width=200)
        droneLocPos.grid(row=1, column=3, columnspan=3,rowspan=2,sticky="NE")

        droneLocPosXLab = tk.Label(droneLocPos, width=10, textvariable=self.droneLocPosXInfo)
        droneLocPosXLab.grid(row=0, column=0, columnspan=1,sticky="NE")
        droneLocPosYLab = tk.Label(droneLocPos, width=10, textvariable=self.droneLocPosYInfo)
        droneLocPosYLab.grid(row=0, column=1, columnspan=1,sticky="NE")
        droneLocPosZLab = tk.Label(droneLocPos, width=10, textvariable=self.droneLocPosZInfo)
        droneLocPosZLab.grid(row=0, column=2, columnspan=1,sticky="NE")
        droneLocRotXLab = tk.Label(droneLocPos, textvariable=self.droneLocRotXInfo)
        droneLocRotXLab.grid(row=1, column=0, columnspan=1,sticky="NE")
        droneLocRotYLab = tk.Label(droneLocPos, textvariable=self.droneLocRotYInfo)
        droneLocRotYLab.grid(row=1, column=1, columnspan=1,sticky="NE")
        droneLocRotZLab = tk.Label(droneLocPos, textvariable=self.droneLocRotZInfo)
        droneLocRotZLab.grid(row=1, column=2, columnspan=1,sticky="NE")
        
        rospy.loginfo("Drone Control Node: Setting GUI Finish")

        #update GUI info
        self.GUIinfoUpdate()
        self.control_node_body()
        self.root.mainloop()

    def GUIinfoUpdate(self):
            #self.droneNavPosXInfo.set('X:'+self.droneLocalPosX)
            #self.droneNavPosYInfo.set('Y:'+self.droneLocalPosY)
            #self.droneNavPosZInfo.set('Z:'+self.droneLocalPosZ)
            #self.droneNavRotXInfo.set('X(deg):'+self.droneLocalRotX)
            #self.droneNavRotYInfo.set('Y(deg):'+self.droneLocalRotY)
            #self.droneNavRotZInfo.set('Z(deg):'+self.droneLocalRotZ)
            #rospy.loginfo("Drone Control Node: local pos X:%f,Y:%f,Z:%f",self.droneLocalPosX,self.droneLocalPosY,self.droneLocalPosZ)
            self.droneLocPosXInfo.set('X(m):%.2f' % (self.droneLocalPosX))
            self.droneLocPosYInfo.set('Y(m):%.2f' % (self.droneLocalPosY))
            self.droneLocPosZInfo.set('Z(m):%.2f' % (self.droneLocalPosZ))
            self.droneLocRotXInfo.set('X(deg):%.2f'% (math.degrees(self.droneLocalRotX)))
            self.droneLocRotYInfo.set('Y(deg):%.2f'% (math.degrees(self.droneLocalRotY)))
            self.droneLocRotZInfo.set('Z(deg):%.2f'% (math.degrees(self.droneLocalRotZ)))

            #contorl mode
            if self.mode == "auto":
                self.manualModeBut.config(state=tk.NORMAL, bg="grey")
                self.autoModeBut.config(state=tk.DISABLED, bg="green")
            elif self.mode == "manual":
                self.manualModeBut.config(state=tk.DISABLED, bg="green")
                self.autoModeBut.config(state=tk.NORMAL, bg= "grey")
            
            #PX4 State
            if self.isPX4_armed:
                self.px4ArmBut.config(bg="green")
            else:
                self.px4ArmBut.config(bg="red")
            
            if self.px4_mode == "POSCTL":
                self.px4PosBut.config(bg="green")
                self.px4OffBoardBut.config(bg="grey")
                self.px4LandBut.config(bg="grey")
                self.px4RTLBut.config(bg="grey")
            elif self.px4_mode == "OFFBOARD":
                self.px4OffBoardBut.config(bg="green")
                self.px4PosBut.config(bg="grey")
                self.px4LandBut.config(bg="grey")
                self.px4RTLBut.config(bg="grey")
            elif self.px4_mode == "AUTO.LAND":
                self.px4OffBoardBut.config(bg="grey")
                self.px4PosBut.config(bg="grey")
                self.px4LandBut.config(bg="green")
                self.px4RTLBut.config(bg="grey")
            elif self.px4_mode == "AUTO.RTL":
                self.px4OffBoardBut.config(bg="grey")
                self.px4PosBut.config(bg="grey")
                self.px4LandBut.config(bg="grey")
                self.px4RTLBut.config(bg="green")
            else:
                self.px4OffBoardBut.config(bg="grey")
                self.px4PosBut.config(bg="grey")
                self.px4LandBut.config(bg="grey")
                self.px4RTLBut.config(bg="grey")


            #rospy.loginfo("Drone Control Node: local pos X:%f,Y:%f,Z:%f",self.droneLocalPosX,self.droneLocalPosY,self.droneLocalPosZ)
            #if not rospy.is_shutdown():
                #self.GUIInfo.start()
            self.root.after(500,self.GUIinfoUpdate)
            

    def start(self):
        #ROS Body
        #while not rospy.is_shutdown():
        # rospy.spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo("Drone Control Node: ROS System Up")
        self.gui()
        #incase tk exit the mainloop somehow
        self.root.destroy()

    def control_node_body(self):
        finalPoseStamped = PoseStamped()
        targetPoseStamped = PoseStamped()
        if self.mode == "manual":
            #rospy.loginfo("Drone Control Node: Manual Mode Running")
            #To MAVROS
            if(self.dronePosSafe):#droneTargetPosSafe =T => have user input in Manual mode
                finalPoseStamped.header.stamp = rospy.Time.now()
                finalPoseStamped.header.frame_id = self.base_link
                finalPoseStamped.pose.position.x = self.droneTargetPosX
                finalPoseStamped.pose.position.y = self.droneTargetPosY
                finalPoseStamped.pose.position.z = self.droneTargetPosZ
                finalPoseStamped.pose.orientation.x = self.droneTargetQuatX
                finalPoseStamped.pose.orientation.y = self.droneTargetQuatY
                finalPoseStamped.pose.orientation.z = self.droneTargetQuatZ
                finalPoseStamped.pose.orientation.w = self.droneTargetQuatW
            else:
                #Output local pos
                finalPoseStamped.header.stamp = rospy.Time.now()
                finalPoseStamped.header.frame_id = self.base_link
                finalPoseStamped.pose.position.x = self.droneSafeLocalPosX
                finalPoseStamped.pose.position.y = self.droneSafeLocalPosY
                finalPoseStamped.pose.position.z = self.droneSafeLocalPosZ
                finalPoseStamped.pose.orientation.x = self.droneSafeLocalQuatX
                finalPoseStamped.pose.orientation.y = self.droneSafeLocalQuatY
                finalPoseStamped.pose.orientation.z = self.droneSafeLocalQuatZ
                finalPoseStamped.pose.orientation.w = self.droneSafeLocalQuatW
            self.dron_position_pub.publish(finalPoseStamped)

            #Current Status
            self.dron_control_mode_pub.publish("manual")
            self.automode_pub.publish(rospy.Time.now(),False)
            #rospy.loginfo("Node:" + finalPoseStamped)
            
        elif self.mode == "auto":
            #rospy.loginfo("Drone Control Node: Auto Mode Running")
            #To MAVROS
            if (self.droneNavPosSafe):#droneNavPosSafe =T => have Navgoation input 
                finalPoseStamped.header.stamp = self.droneNavtime
                finalPoseStamped.header.frame_id = self.base_link
                finalPoseStamped.pose.position.x = self.droneNavPosX
                finalPoseStamped.pose.position.y = self.droneNavPosY
                finalPoseStamped.pose.position.z = self.droneNavPosZ
                finalPoseStamped.pose.orientation.x = self.droneNavQuatX
                finalPoseStamped.pose.orientation.y = self.droneNavQuatY
                finalPoseStamped.pose.orientation.z = self.droneNavQuatZ
                finalPoseStamped.pose.orientation.w = self.droneNavQuatW
            else:
                #Output local pos
                finalPoseStamped.header.stamp = rospy.Time.now()
                finalPoseStamped.header.frame_id = self.base_link
                finalPoseStamped.pose.position.x = self.droneSafeLocalPosX
                finalPoseStamped.pose.position.y = self.droneSafeLocalPosY
                finalPoseStamped.pose.position.z = self.droneSafeLocalPosZ
                finalPoseStamped.pose.orientation.x = self.droneSafeLocalQuatX
                finalPoseStamped.pose.orientation.y = self.droneSafeLocalQuatY
                finalPoseStamped.pose.orientation.z = self.droneSafeLocalQuatZ
                finalPoseStamped.pose.orientation.w = self.droneSafeLocalQuatW
            self.dron_position_pub.publish(finalPoseStamped)
            
            # For nagvation use
            if (self.droneTargetPosSafe):#droneNavPosSafe =T => have user input in auto mode
                targetPoseStamped.header.stamp = rospy.Time.now()
                targetPoseStamped.header.frame_id = self.base_link
                targetPoseStamped.pose.position.x = self.droneTargetPosX
                targetPoseStamped.pose.position.y = self.droneTargetPosY
                targetPoseStamped.pose.position.z = self.droneTargetPosZ
                targetPoseStamped.pose.orientation.x = self.droneTargetQuatX
                targetPoseStamped.pose.orientation.y = self.droneTargetQuatY
                targetPoseStamped.pose.orientation.z = self.droneTargetQuatZ
                targetPoseStamped.pose.orientation.w = self.droneTargetQuatW
                self.dron_nagvation_pose_pub.publish(targetPoseStamped)
            else:
                rospy.loginfo("Drone Control Node: Wiating Hei Input...")
                #Output local pos
                #targetPoseStamped.header.stamp = rospy.Time.now()
                #targetPoseStamped.header.frame_id = self.base_link
                #targetPoseStamped.pose.position.x = self.droneLocalPosX
                #targetPoseStamped.pose.position.y = self.droneLocalPosY
                #targetPoseStamped.pose.position.z = self.droneLocalPosZ
                #targetPoseStamped.pose.orientation.x = self.droneLocalQuatX
                #targetPoseStamped.pose.orientation.y = self.droneLocalQuatY
                #targetPoseStamped.pose.orientation.z = self.droneLocalQuatZ
                #targetPoseStamped.pose.orientation.w = self.droneLocalQuatW
            #self.dron_nagvation_pose_pub.publish(targetPoseStamped)

            #Current Status
            self.dron_control_mode_pub.publish("auto")
            self.automode_pub.publish(rospy.Time.now(),True)
            #rospy.loginfo(finalPoseStamped)
            
        else:
            self.mode = "manual"
        self.root.after(2,self.control_node_body)


        

if __name__ == '__main__':
    my_node = drone_control_node()
    #my_node.GUIinfoUpdateTh()
    #Start GUI part
    #GUIbody = threading.Thread(target = my_node.gui(),daemon = True)
    #Thread to update info from ROS to GUI
    #GUIbody.start()

    #ControlBody = threading.Thread(target = my_node.control_node_body(),daemon = True)
    #ControlBody.start()
    ##start ROS
    my_node.start()


