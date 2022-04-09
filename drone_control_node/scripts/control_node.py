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
import time
import threading
from multiprocessing import Process
#import tkthread; tkthread.tkinstall()
#from tkthread import tk
import tkinter as tk
import tkinter.font as tkfont

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from drone_control_msgs.msg import GetDroneState,SetDroneState
#from std_msgs.msg import Header 

# for navigation node message filters use
from jsk_recognition_msgs.msg import BoolStamped  

# for image 
from sensor_msgs.msg import Image
import PIL.Image
import PIL.ImageTk

# gripper
try:
    from ros_circuitpython_servokit_msgs.msg import AllServoAngle
except:
    pass
class drone_control_node(object):
    
    def __init__(self):
        # Tells rospy the name of the node.
        # Anonymous = True makes sure the node has a unique name. Random
        # numbers are added to the end of the name.
        rospy.init_node("drone_control", anonymous=True, disable_signals=True)

        # ROS loop rate
        # Rate
        self.loop_rate = rospy.Rate(40)
        
        # Get param
        self.isOnboard = rospy.get_param('~onboard')
        self.needImage = rospy.get_param('~image')
        self.hasGripper = rospy.get_param('~hasGripper')
        if self.hasGripper:
            try:
                from ros_circuitpython_servokit_msgs.msg import AllServoAngle
            except:
                rospy.logerr("Drone Control Node: Have you install ROS_CircuitPython_ServoKit?")
                rospy.logerr("Drone Control Node: Without gripper counld launch with")
                self.hasGripper = False
            else:
                self.dron_servo_pub = rospy.Publisher("/servo/angle", AllServoAngle, queue_size=1)
        if self.isOnboard:
            rospy.loginfo("Drone Control Node: Onboard")
            #rospy.loginfo(self.isOnboard)
        else:
            rospy.loginfo("Drone Control Node: Offboard")

        # Node is publishing to the topic
        # e.g. self.vesc1_pub = rospy.Publisher(vesc1_ns + '/commands/motor/speed', Float64, queue_size=10)
        if self.isOnboard:
            #ros topic for onboard
            self.dron_position_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
            self.dron_control_mode_pub = rospy.Publisher("/drone/current/control_mode", String, queue_size=1)
            self.dron_nagvation_pose_pub = rospy.Publisher("/drone/nagvation/pos", PoseStamped, queue_size=1)
            self.automode_pub = rospy.Publisher("/auto_mode/status", BoolStamped, queue_size=1)
            self.current_control_status_pub = rospy.Publisher("/drone/current/control_status", GetDroneState, queue_size=1)

            # Node subscribing topics
            # Subscribe to drone state
            self.target_pos_sub = message_filters.Subscriber("/drone/input_postion/pose", PoseStamped)
            self.target_pos_sub.registerCallback(self.callback_target_position)
            #self.target_ground_pos_sub = message_filters.Subscriber("/drone/ground_control/pose", PoseStamped)
            #self.target_ground_pos_sub.registerCallback(self.callback_target_ground_position)
            self.target_control_status_sub = message_filters.Subscriber("/drone/set/control_status",SetDroneState)
            self.target_control_status_sub.registerCallback(self.callback_set_control_status)
            #ros topic common
            self.local_pos_sub = message_filters.Subscriber("/mavros/local_position/pose", PoseStamped)
            self.local_pos_sub.registerCallback(self.callback_local_position)
            self.px4_state_sub = message_filters.Subscriber("/mavros/state", State)
            self.px4_state_sub.registerCallback(self.callback_px4_state)
        else:
            #ros topic for offboard
            self.current_control_status_pub = rospy.Publisher("/drone/set/control_status", SetDroneState, queue_size=1)
            # Node subscribing topics
            self.target_control_status_sub = message_filters.Subscriber("/drone/current/control_status",GetDroneState)
            self.target_control_status_sub.registerCallback(self.callback_get_control_status)
            #self.local_pos_sub = message_filters.Subscriber("/drone/current/control_mode", String)

        if self.needImage:
            self.image_sub = message_filters.Subscriber("/mavros/state", Image)
            self.image_sub.registerCallback(self.callback_image)
            self.tkImage = PIL.ImageTk.PhotoImage()
        
        #self.base_link="base_link"
        self.base_link= rospy.get_param('~base_link')
        
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

        # px4 state
        self.isPX4_armed= False
        self.isPX4_connected= False
        self.px4_mode= ""
        self.px4ConnectionInfo = tk.StringVar(self.root, value="Disconnected!")

        # drone step
        self.verticalStep= 1.0
        self.horizontalStep= 1.0
        self.rotationalStep= 45.0

        # remote
        self.currentTime = rospy.Time()
        self.allowRemote = rospy.get_param('~allow_remote')
        #self.armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        
        # For offboard use  
        self.off_status_Time = rospy.Time()

        # servo related
        self.servoAngle= tk.DoubleVar()

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

    def callback_target_ground_position(self, msg):
        if self.mode== "remote":
            #target POS by Navgoation
            self.droneTargetPosX =msg.pose.position.x
            self.droneTargetPosY =msg.pose.position.y
            self.droneTargetPosZ =msg.pose.position.z
            #self.droneNavtime =msg.header.stamp
            #-For rotate only yaw should be change
            #rospy.loginfo(msg.pose.orientation)
            self.droneTargetQuatX = msg.pose.orientation.x
            self.droneTargetQuatY = msg.pose.orientation.y
            self.droneTargetQuatZ = msg.pose.orientation.z
            self.droneTargetQuatW = msg.pose.orientation.w
            self.droneTargetPosSafe = True
            self.dronePosSafe = True

    #callback_px4_postion(input)
    def callback_px4_state(self, msg):
        self.isPX4_armed= msg.armed
        self.isPX4_connected= msg.connected
        self.px4_mode= msg.mode
    
    #onboard node handle offboard node request 
    def callback_set_control_status(self, msg):
        #time filter only accept the lastest ros message
        rospy.loginfo(msg)
        if msg.header.stamp > self.currentTime:
            self.currentTime= msg.header.stamp
            if self.allowRemote:
                if not(msg.useCurrentPx4Mode):
                    self.modeDaemon = threading.Thread(target = self.px4SetMode(msg.setPx4mode),daemon = True)
                    self.modeDaemon.start()
                if not(msg.useCurrentMode):
                    if msg.controlMode == GetDroneState.MODE_CONTROL_MODE_MANUAL:
                        if msg.controlMode != self.mode:
                            self.manualMode()
                    elif msg.controlMode == GetDroneState.MODE_CONTROL_MODE_AUTO:
                        if msg.controlMode != self.mode:
                            self.autoMode()
                if not(msg.useLocalPos):
                    self.droneTargetPosX = msg.setTargetPos.position.x
                    self.droneTargetPosY = msg.setTargetPos.position.y
                    self.droneTargetPosZ = msg.setTargetPos.position.z
                    self.droneTargetQuatX = msg.setTargetPos.orientation.x
                    self.droneTargetQuatY = msg.setTargetPos.orientation.y
                    self.droneTargetQuatZ = msg.setTargetPos.orientation.z
                    self.droneTargetQuatW = msg.setTargetPos.orientation.w
                    self.droneTargetPosSafe = True
                    self.dronePosSafe = True
                self.servoAngleScale.set(msg.servoAngle)
    
    #image
    def callback_image(self,msg):
        tempimage = PIL.Image.fromarray(msg.data)
        self.tkImage = PIL.ImageTk.PhotoImage(tempimage)
        self.imageLab.configure(image=self.tkImage)
        self.imageLab.image = self.tkImage

    #for offboard
    def callback_get_control_status(self,msg):
        if msg.header.stamp > self.off_status_Time:
            self.off_status_Time = msg.header.stamp
            self.isPX4_connected = msg.isPx4Connected
            self.isPX4_armed = msg.isPx4Armed
            self.px4_mode = msg.getPx4mode
            self.droneLocalPosX =msg.getPx4LocalPos.position.x
            self.droneLocalPosY =msg.getPx4LocalPos.position.y
            self.droneLocalPosZ =msg.getPx4LocalPos.position.z
            orientation_quaternion = msg.getPx4LocalPos.orientation
            orientation_quaternion_list = [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w]
            #(roll, pitch, yaw)
            (self.droneLocalRotX, self.droneLocalRotY, self.droneLocalRotZ) = euler_from_quaternion (orientation_quaternion_list)
            self.droneLocalQuatX = msg.getPx4LocalPos.orientation.x
            self.droneLocalQuatY = msg.getPx4LocalPos.orientation.y
            self.droneLocalQuatZ = msg.getPx4LocalPos.orientation.z
            self.droneLocalQuatW = msg.getPx4LocalPos.orientation.w
                    


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


    #-Allow/Not Allow remote
    def remoteMode(self):
        self.changeModeSafety()
        if self.allowRemote:
            self.notAllowRemoteMode()
            rospy.loginfo("Drone Control Node: Setting Control to Allow Remote Control....")
        else:
            self.allowRemoteMode()
            rospy.loginfo("Drone Control Node: Setting Control to NOT Allow Remote Control....")

    #--Allow Remote
    def allowRemoteMode(self):
        self.allowRemote= True
    def notAllowRemoteMode(self):
        self.allowRemote= False

    #-manual
    #--GUI set px4 target pos
    def manualMode(self):
        self.changeModeSafety()
        self.mode="manual"
        rospy.loginfo("Drone Control Node: Setting Control to Manual....")


    #PX4 Mode
    #-arm PX4
    def setPx4ArmMode(self):
        if self.isPX4_armed:
            # if arm already trying to disarm
            rospy.loginfo("Drone Control Node: Trying PX4 Mode to Disarm....")
            #rospy.wait_for_service('mavros/cmd/arming')
            try:
                self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                self.armService(False)
            except rospy.ServiceException as e:
                rospy.logwarn("[Drone Control Node] Disarm service failed: %s"%e)
        else:
            # if disarm already trying to arm
            rospy.loginfo("Drone Control Node: Trying PX4 Mode to Arm....")
            #rospy.wait_for_service('mavros/cmd/arming')
            try:
                self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                self.armService(True)
            except rospy.ServiceException as e:
                rospy.logwarn("[Drone Control Node] Aming service failed: %s"%e)
    def px4ArmMode(self):
        rospy.loginfo("Drone Control Node: Starting PX4 Mode to Arm")
        self.changeModeSafety()
        self.armDaemon = threading.Thread(target = self.setPx4ArmMode()).start()
        #self.armDaemon = threading.Thread(target = self.setPx4ArmMode(),daemon = True).start()
        #self.armDaemon = Process(target = self.setPx4ArmMode(),daemon = True)
        #self.armDaemon.start()

    #-px4 setmode
    def px4SetMode(self, setToMode):
        rospy.loginfo("Drone Control Node: Trying to set PX4 Mode to %s",setToMode)
        #rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            flightModeService(custom_mode= setToMode)
        except rospy.ServiceException as e:
            rospy.logwarn("[Drone Control Node] Aming service failed: %s"%e)

    #-px4 to Position Mode
    def px4PosMode(self):
        #later do
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to Position Mode....")
        self.modeDaemon = threading.Thread(target = self.px4SetMode(State.MODE_PX4_POSITION),daemon = True)
        self.modeDaemon.start()

    #-px4 to OffBoard Mode
    def px4OffBoardMode(self):
        #later do
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to Offboard Mode")
        self.modeDaemon = threading.Thread(target = self.px4SetMode(State.MODE_PX4_OFFBOARD),daemon = True)
        self.modeDaemon.start()

    def px4LandMode(self):
        #later do
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to Auto Land Mode")
        self.modeDaemon = threading.Thread(target = self.px4SetMode(State.MODE_PX4_LAND),daemon = True)
        self.modeDaemon.start()

    def px4RTLMode(self):
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to AUTO.RTL(Return to Start) Mode")
        self.modeDaemon = threading.Thread(target = self.px4SetMode(State.MODE_PX4_RTL),daemon = True)
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
    def changeDroneDir(self,Xc,Yc):
        # atan2Angle = math.atan2(self.droneLocalPosY,self.droneLocalPosX)
        atan2Angle = self.droneLocalRotZ
        final_x= self.droneLocalPosX + Xc*math.cos(atan2Angle) - Yc*math.sin(atan2Angle)
        final_y= self.droneLocalPosY + Xc*math.sin(atan2Angle) + Yc*math.cos(atan2Angle)
        self.setDronPosXYZ(final_x,final_y,self.droneLocalPosZ,math.degrees(self.droneLocalRotZ))
        rospy.loginfo("X:%.2f, Y:%.2f atan: %.2f " % (final_x,final_y, math.degrees(atan2Angle)))

    def droneForward(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Forward")
        #self.setDronPosXYZ(self.droneLocalPosX+self.horizontalStep,self.droneLocalPosY,self.droneLocalPosZ,math.degrees(self.droneLocalRotZ))
        self.changeDroneDir(self.horizontalStep,0)
    
    def droneLeft(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Left")
        #elf.setDronPosXYZ(self.droneLocalPosX,self.droneLocalPosY+self.horizontalStep,self.droneLocalPosZ,math.degrees(self.droneLocalRotZ))
        self.changeDroneDir(0,self.horizontalStep)
    
    def droneStop(self):
        #later do
        self.setDronPosXYZ(self.droneLocalPosX,self.droneLocalPosY,self.droneLocalPosZ,math.degrees(self.droneLocalRotZ))
        rospy.loginfo("Drone Control Node: Drone Stop")
        

    def droneRight(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Right")
        #self.setDronPosXYZ(self.droneLocalPosX,self.droneLocalPosY-self.horizontalStep,self.droneLocalPosZ,math.degrees(self.droneLocalRotZ))
        self.changeDroneDir(0,-1* self.horizontalStep)
    
    def droneBackward(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Backward")
        #self.setDronPosXYZ(self.droneLocalPosX-self.horizontalStep,self.droneLocalPosY,self.droneLocalPosZ,math.degrees(self.droneLocalRotZ))
        self.changeDroneDir(-1* self.horizontalStep,0)

    #-Vertical Movement
    def droneUp(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Moving Up")
        self.setDronPosXYZ(self.droneLocalPosX,self.droneLocalPosY,self.droneLocalPosZ+self.verticalStep,math.degrees(self.droneLocalRotZ))
    
    def droneDown(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Moving Down")
        self.setDronPosXYZ(self.droneLocalPosX,self.droneLocalPosY,self.droneLocalPosZ-self.verticalStep,math.degrees(self.droneLocalRotZ))

    def droneRotateLeft(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Rotating Left")
        self.setDronPosXYZ(self.droneLocalPosX,self.droneLocalPosY,self.droneLocalPosZ,math.degrees(self.droneLocalRotZ)+self.rotationalStep)
    
    def droneRotateRight(self):
        #later do
        rospy.loginfo("Drone Control Node: Drone Rotating Right")
        self.setDronPosXYZ(self.droneLocalPosX,self.droneLocalPosY,self.droneLocalPosZ,math.degrees(self.droneLocalRotZ)-self.rotationalStep)

    def takeLocal(self):
        self.dronePosXInp.set(round(self.droneLocalPosX,3))
        self.dronePosYInp.set(round(self.droneLocalPosY,3))
        self.dronePosZInp.set(round(self.droneLocalPosZ,3))
        self.droneRotZInp.set(round(math.degrees(self.droneLocalRotZ),3))
    
    def stopmanual(self):
        self.manualMode()
        self.setDronPosXYZ(self.droneLocalPosX,self.droneLocalPosY,self.droneLocalPosZ,math.degrees(self.droneLocalRotZ))


    #GUI Body
    def gui(self):
        rospy.loginfo("Drone Control Node: Setting PX4 Mode up GUI now")
        #self.root.destroy()
        #self.root = tk.Tk()
        if self.isOnboard:
            self.root.title("Control Node GUI: Onboard")
        else:
            self.root.title("Control Node GUI: Offboard")
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
        if self.isOnboard:
            title = tk.Label(titlef, text="Control Node: Onboard GUI", font=title_font, anchor="e" )
        else:
            title = tk.Label(titlef, text="Control Node: Offboard GUI", font=title_font, anchor="e" )
        title.grid(row=0, column=1,sticky="")
        titlef.grid(row=0, column=0, columnspan=6,sticky="")

        if self.needImage:
            self.imageLab= tk.Label(self.root, image=self.tkImage)
            self.imageLab.image = self.tkImage
            title.grid(row=1, columnspan=5)
        
        #Select Control Mode
        target = tk.LabelFrame(self.root, text="Target Control Mode",width=200)
        target.grid(row=2, column=0, columnspan=3,sticky="W")

        self.manualModeBut = tk.Button(target, text="Manual", width=10,bd=2, cursor="exchange", command = lambda: self.manualMode())
        self.manualModeBut.grid(row=1, column=0, columnspan=1)

        self.autoModeBut = tk.Button(target, text="Auto", width=10,bd=2, cursor="exchange", command = lambda: self.autoMode())
        self.autoModeBut.grid(row=1, column=1, columnspan=1) 

        if self.isOnboard:
            self.remoteControlBut = tk.Button(target, text="Remote", width=10,bd=2, cursor="exchange", command = lambda: self.remoteMode())
            self.remoteControlBut.grid(row=1, column=2, columnspan=1)
        
        #Display Local Pos
        droneLocPos = tk.LabelFrame(self.root, text="Current Position:",width=200)
        droneLocPos.grid(row=2, column=3, columnspan=3,rowspan=1,sticky="NW")

        droneLocPosXLab = tk.Label(droneLocPos, width=10, textvariable=self.droneLocPosXInfo)
        droneLocPosXLab.grid(row=0, column=0, columnspan=1,sticky="NW")
        droneLocPosYLab = tk.Label(droneLocPos, width=10, textvariable=self.droneLocPosYInfo)
        droneLocPosYLab.grid(row=0, column=1, columnspan=1,sticky="NW")
        droneLocPosZLab = tk.Label(droneLocPos, width=10, textvariable=self.droneLocPosZInfo)
        droneLocPosZLab.grid(row=0, column=2, columnspan=1,sticky="NW")
        droneLocRotXLab = tk.Label(droneLocPos, textvariable=self.droneLocRotXInfo)
        droneLocRotXLab.grid(row=1, column=0, columnspan=1,sticky="NW")
        droneLocRotYLab = tk.Label(droneLocPos, textvariable=self.droneLocRotYInfo)
        droneLocRotYLab.grid(row=1, column=1, columnspan=1,sticky="NW")
        droneLocRotZLab = tk.Label(droneLocPos, textvariable=self.droneLocRotZInfo)
        droneLocRotZLab.grid(row=1, column=2, columnspan=1,sticky="NW")

        #Select Px4 Flight Mode
        self.px4Mode = tk.LabelFrame(self.root, text="PX4 Flight Mode",width=200)
        self.px4Mode.grid(row=3, column=0, columnspan=5,rowspan=2, sticky="N")

        # -px4 mode row 0 
        self.px4StatusLab = tk.Label(self.px4Mode, text="PX4:")
        self.px4StatusLab.grid(row=0, column=0, columnspan=1,sticky="NE")

        self.px4ConnectionLab = tk.Label(self.px4Mode, textvariable=self.px4ConnectionInfo)
        self.px4ConnectionLab.grid(row=0, column=1, columnspan=2,sticky="NW")

        # -px4 mode row 1 
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
        droneMovement.grid(row=5, column=0, columnspan=10,sticky="W")
        
        #Horizontal Movement
        droneHorizontalMovement = tk.LabelFrame(droneMovement, text="Horizontal",width=200)
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
        droneVerticalMovement = tk.LabelFrame(droneMovement, text="Vertical",width=200)
        droneVerticalMovement.grid(row=0, column=3, columnspan=1,rowspan=3,sticky="N")

        droneUpBut = tk.Button(droneVerticalMovement, text="Up", width=10,bd=2, cursor="exchange", command = lambda: self.droneUp())
        droneUpBut.grid(row=0, column=0, columnspan=1) 

        droneStop2But = tk.Button(droneVerticalMovement, text="Hold", width=10,bd=2, cursor="exchange", command = lambda: self.droneStop())
        droneStop2But.grid(row=1, column=0, columnspan=1)

        droneDownBut = tk.Button(droneVerticalMovement, text="Down", width=10,bd=2, cursor="exchange", command = lambda: self.droneDown())
        droneDownBut.grid(row=2, column=0, columnspan=1) 

        #Rotational Movement
        droneRotationalMovement = tk.LabelFrame(droneMovement, text="Rotational",width=200)
        droneRotationalMovement.grid(row=0, column=4, columnspan=2,rowspan=3,sticky="N")

        droneRotateLeftBut = tk.Button(droneRotationalMovement, text="Rotate Left", width=10,bd=2, cursor="exchange", command = lambda: self.droneRotateLeft())
        droneRotateLeftBut.grid(row=0, column=0, columnspan=1) 

        droneRotateRightBut = tk.Button(droneRotationalMovement, text="Rotate Right", width=10,bd=2, cursor="exchange", command = lambda: self.droneRotateRight())
        droneRotateRightBut.grid(row=0, column=1, columnspan=1)

        #Drone Movement by Pos
        dronePosMovement = tk.LabelFrame(self.root, text="Drone Movement(By Position)",width=200)
        dronePosMovement.grid(row=8, column=0, columnspan=10,rowspan=2,sticky="W")
        
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


        # Gripper Movement by Servo Value
        if self.hasGripper:
            self.gripperMovement = tk.LabelFrame(self.root, text="Gripper Movement(By Servo Angle)",width=200)
            self.gripperMovement.grid(row=10, column=0, columnspan=10,rowspan=1,sticky="W")
            self.servoAngleScale=tk.Scale( self.gripperMovement, variable = self.servoAngle, from_ = 0, to = 180, orient = tk.HORIZONTAL, width=10,length=200) 
            self.servoAngleScale.grid(row=0,column=0, columnspan=3)

        
        rospy.loginfo("Drone Control Node: Setting GUI Finish")

        #update GUI info
        if (self.isOnboard):
            self.remote_info_update()
            self.control_node_body()
        else:
            self.control_node_body_offboard()
        self.GUIinfoUpdate()
        self.root.mainloop()

    def GUIinfoUpdate(self):
            #self.droneNavPosXInfo.set('X:'+self.droneLocalPosX)
            #self.droneNavPosYInfo.set('Y:'+self.droneLocalPosY)
            #self.droneNavPosZInfo.set('Z:'+self.droneLocalPosZ)
            #self.droneNavRotXInfo.set('X(deg):'+self.droneLocalRotX)
            #self.droneNavRotYInfo.set('Y(deg):'+self.droneLocalRotY)
            #self.droneNavRotZInfo.set('Z(deg):'+self.droneLocalRotZ)
            #rospy.loginfo("Drone Control Node: local pos X:%f,Y:%f,Z:%f",self.droneLocalPosX,self.droneLocalPosY,self.droneLocalPosZ)
            
            #update local pos
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
            if self.isPX4_connected:
                self.px4ConnectionInfo.set("Connected!")
                self.px4ConnectionLab.config(foreground="white", bg="green")
            else:
                self.px4ConnectionInfo.set("Disconnected!")
                self.px4ConnectionLab.config(foreground="white", bg="red")

            if self.isPX4_armed:
                self.px4ArmBut.config(bg="green")
            else:
                self.px4ArmBut.config(bg="red")
                self.changeModeSafety()
            
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
            
            # Remote Status
            if self.isOnboard:
                if self.allowRemote:
                    self.remoteControlBut.config(bg="green")
                else:
                    self.remoteControlBut.config(bg="red")
            
            # update Servo Gripper
            if self.hasGripper and self.isOnboard:
                servo= AllServoAngle()
                servo.header.stamp=rospy.Time.now()
                servo.header.frame_id = self.base_link
                servo.all16servoPWM=[float(self.servoAngle.get())]*16
                self.dron_servo_pub.publish(servo)

            
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
    
    def remote_info_update(self):
        remotDroneState = GetDroneState()
        remotDroneState.header.stamp = rospy.Time.now()
        remotDroneState.header.frame_id = self.base_link
        remotDroneState.isPx4Connected = self.isPX4_connected
        remotDroneState.isPx4Armed = self.isPX4_armed
        remotDroneState.getPx4mode = self.px4_mode
        remotDroneState.getPx4LocalPos.position.x = self.droneLocalPosX
        remotDroneState.getPx4LocalPos.position.y = self.droneLocalPosY
        remotDroneState.getPx4LocalPos.position.z = self.droneLocalPosZ
        remotDroneState.getPx4LocalPos.orientation.x = self.droneLocalQuatX
        remotDroneState.getPx4LocalPos.orientation.y = self.droneLocalQuatY
        remotDroneState.getPx4LocalPos.orientation.z = self.droneLocalQuatZ
        remotDroneState.getPx4LocalPos.orientation.w = self.droneLocalQuatW
        remotDroneState.controlMode = self.mode
        self.current_control_status_pub.publish(remotDroneState)
        self.root.after(100,self.remote_info_update)




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
            self.dron_nagvation_pose_pub.publish(finalPoseStamped)
            #Current Status
            self.dron_control_mode_pub.publish("manual")
            status = BoolStamped()
            status.header.stamp = rospy.Time.now()
            status.data=False
            self.automode_pub.publish(status)
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
            status = BoolStamped()
            status.header.stamp = rospy.Time.now()
            status.data=True
            self.automode_pub.publish(status)
            #rospy.loginfo(finalPoseStamped)
            
        else:
            self.mode = "manual"
        self.root.after(2,self.control_node_body)

    def control_node_body_offboard(self):
        targetState = SetDroneState()
        if self.mode == "manual":
            #rospy.loginfo("Drone Control Node: Manual Mode Running")
            #To MAVROS
            if(self.dronePosSafe):#droneTargetPosSafe =T => have user input in Manual mode
                targetState.header.stamp = rospy.Time.now()
                targetState.header.frame_id = self.base_link
                targetState.useCurrentPx4Mode=True
                targetState.useCurrentMode=False
                targetState.controlMode="manual"
                targetState.useLocalPos=False
                targetState.setTargetPos.position.x = self.droneTargetPosX
                targetState.setTargetPos.position.y = self.droneTargetPosY
                targetState.setTargetPos.position.z = self.droneTargetPosZ
                targetState.setTargetPos.orientation.x = self.droneTargetQuatX
                targetState.setTargetPos.orientation.y = self.droneTargetQuatY
                targetState.setTargetPos.orientation.z = self.droneTargetQuatZ
                targetState.setTargetPos.orientation.w = self.droneTargetQuatW
                targetState.servoAngle = float(self.servoAngle.get())
            else:
                #Output local pos
                targetState.header.stamp = rospy.Time.now()
                targetState.header.frame_id = self.base_link
                targetState.useCurrentPx4Mode=True
                targetState.useCurrentMode=False
                targetState.controlMode="manual"
                targetState.useLocalPos=True
                targetState.servoAngle = float(self.servoAngle.get())
            self.current_control_status_pub.publish(targetState)
            
        elif self.mode == "auto":
            if(self.dronePosSafe):#droneTargetPosSafe =T => have user input in Manual mode
                targetState.header.stamp = rospy.Time.now()
                targetState.header.frame_id = self.base_link
                targetState.useCurrentPx4Mode=True
                targetState.useCurrentMode=False
                targetState.controlMode="auto"
                targetState.useLocalPos=False
                targetState.setTargetPos.position.x = self.droneTargetPosX
                targetState.setTargetPos.position.y = self.droneTargetPosY
                targetState.setTargetPos.position.z = self.droneTargetPosZ
                targetState.setTargetPos.orientation.x = self.droneTargetQuatX
                targetState.setTargetPos.orientation.y = self.droneTargetQuatY
                targetState.setTargetPos.orientation.z = self.droneTargetQuatZ
                targetState.setTargetPos.orientation.w = self.droneTargetQuatW
                targetState.servoAngle = float(self.servoAngle.get())
            else:
                #Output local pos
                targetState.header.stamp = rospy.Time.now()
                targetState.header.frame_id = self.base_link
                targetState.useCurrentPx4Mode=True
                targetState.useCurrentMode=False
                targetState.controlMode="auto"
                targetState.useLocalPos=True
                targetState.servoAngle = float(self.servoAngle.get())
            self.current_control_status_pub.publish(targetState)
            
        else:
            self.mode = "manual"
        self.root.after(20,self.control_node_body_offboard)


        

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
