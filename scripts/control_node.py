#!/usr/bin/env python 3

import rospy
import message_filters # To Achieve Multiple subscriber
#from std_msgs.msg import Float32
#from std_msgs.msg import Float64
#from std_msgs.msg import String
import threading
import tkinter as tk
import tkinter.font as tkfont
import time

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class drone_control_node(object):
    
    def __init__(self):

        # ROS loop rate

        # Tells rospy the name of the node.
        # Anonymous = True makes sure the node has a unique name. Random
        # numbers are added to the end of the name.
        rospy.init_node("state_machine_node", anonymous=True, disable_signals=True)

        # Rate
        self.loop_rate = rospy.Rate(60)

        # Node is subscribing to the topic
        # Subscribe to drone state
        #rospy.Subscriber('mavros/state', State, self.stateCb)
        #rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.posCb)
        sub = message_filters.Subscriber("mavros/local_position/pose", PoseStamped)
        sub.registerCallback(self.callback_local_position)
        sub = message_filters.Subscriber("drone/target_postion/pose", PoseStamped)
        sub.registerCallback(self.callback_target_position)

        # Node is publishing to the topic
        #self.vesc1_pub = rospy.Publisher(vesc1_ns + '/commands/motor/speed', Float64, queue_size=10)

        #setup variable
        self.mode="manual"

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


        #target POS by GUI manual input
        self.droneTargetPosX =0.0
        self.droneTargetPosY =0.0
        self.droneTargetPosZ =0.0
        #-For rotate only yaw should be change
        self.droneTargetQuatX =0.0
        self.droneTargetQuatY =0.0
        self.droneTargetQuatZ =0.0
        self.droneTargetQuatW =0.0


        #target POS by Navgoation
        self.droneNavPosX =0.0
        self.droneNavPosY =0.0
        self.droneNavPosZ =0.0
        #-For rotate only yaw should be change
        self.droneNavQuatX =0.0
        self.droneNavQuatY =0.0
        self.droneNavQuatZ =0.0
        self.droneNavQuatW =0.0

        self.state = State()


    #callback_local_position
    def callback_local_position(self, msg):
        self.droneLocalPosX =msg.pose.position.x
        self.dronePdroneLocalPosYosY =msg.pose.position.y
        self.droneLocalPosZ =msg.pose.position.z
        orientation_quaternion = msg.pose.orientation
        orientation_quaternion_list = [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w]
        #(roll, pitch, yaw)
        (self.droneLocalRotX, self.droneLocalRotY, self.droneLocalRotZ) = euler_from_quaternion (orientation_quaternion_list)

    #callback_target_position
    def callback_target_position(self, msg):
        #target POS by Navgoation
        self.droneNavPosX =msg.pose.position.x
        self.droneNavPosY =msg.pose.position.x
        self.droneNavPosZ =msg.pose.position.x
        #-For rotate only yaw should be change
        (self.droneNavQuatX ,self.droneNavQuatY, self.droneNavQuatZ, self.droneNavQuatW) = msg.pose.orientation

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
    #-Vision
    def autoMode(self):
        self.mode="auto"
        rospy.loginfo("Drone Control Node: Setting Control to Auto(?) Position....")

    #-Pos
    def visionPosMode(self):
        self.mode="visionPos"
        rospy.loginfo("Drone Control Node: Setting Control to Vision Position....")

    #-manual
    #--GUI set px4 target pos
    def manualMode(self):
        self.mode="manual"
        rospy.loginfo("Drone Control Node: Setting Control to Manual....")


    #PX4 Mode
    #-arm PX4
    def setPx4ArmMode(self):
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to Arm....")
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            rospy.logwarn("[Drone Control Node] Aming service failed: %s"%e)
    def px4ArmMode(self):
        armDaemon = threading.Thread(target = self.setPx4ArmMode,daemon = True)

    #-px4 setmode
    def px4SetMode(self, setToMode):
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to %s",setToMode)
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
        modeDaemon = threading.Thread(target = self.px4SetMode('POSITION'),daemon = True)

    #-px4 to OffBoard Mode
    def px4OffBoardMode(self):
        #later do
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to Offboard Mode")
        modeDaemon = threading.Thread(target = self.px4SetMode('OFFBOARD'),daemon = True)


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


    #GUI Body
    def gui(self):
        rospy.loginfo("Drone Control Node: Setting PX4 Mode up GUI now")
        root = tk.Tk()
        root.title("Control Node GUI")

        #title 
        titlef = tk.Frame(root)
        menubar = tk.Menu(root)
        menuList = tk.Menu(menubar, tearoff=0)
        menuList.add_command(label="Exit", command=lambda:root.destroy())
        menubar.add_cascade(label="Menu", menu=menuList)
        root.config(menu=menubar)

        #title style
        title_font = tkfont.Font(family='Helvetica', size=24, weight="bold", slant="italic")

        #title body
        title = tk.Label(titlef, text="Control Node: GUI", font=title_font, anchor="e" )
        title.grid(row=0, column=1,sticky="")
        titlef.grid(row=0, column=0, columnspan=5,sticky="")

        #Select Control Mode
        target = tk.LabelFrame(root, text="Target Control Mode",width=200)
        target.grid(row=1, column=0, columnspan=5,sticky="W")

        autoModeBut = tk.Button(target, text="Auto", width=10,bd=2, cursor="exchange", command = lambda: self.autoMode())
        autoModeBut.grid(row=1, column=0, columnspan=1) 

        semiHardModeBut = tk.Button(target, text="Vision Pos", width=10,bd=2, cursor="exchange", command = lambda: self.visionPosMode())
        semiHardModeBut.grid(row=1, column=1, columnspan=1)

        manualModeBut = tk.Button(target, text="Manual", width=10,bd=2, cursor="exchange", command = lambda: self.manualMode())
        manualModeBut.grid(row=1, column=2, columnspan=1)
        
        #Select Px4 Flight Mode
        px4Mode = tk.LabelFrame(root, text="PX4 Flight Mode",width=200)
        px4Mode.grid(row=2, column=0, columnspan=5,sticky="W")

        px4PosBut = tk.Button(px4Mode, text="Arm", width=10,bd=2, cursor="exchange", command = lambda: self.px4ArmMode())
        px4PosBut.grid(row=1, column=0, columnspan=1) 

        px4PosBut = tk.Button(px4Mode, text="Position Mode", width=10,bd=2, cursor="exchange", command = lambda: self.px4PosMode())
        px4PosBut.grid(row=1, column=1, columnspan=1) 

        px4OffBoardBut = tk.Button(px4Mode, text="Offboard Mode", width=10,bd=2, cursor="exchange", command = lambda: self.px4OffBoardMode())
        px4OffBoardBut.grid(row=1, column=2, columnspan=2) 

        #Drone Movement
        droneMovement = tk.LabelFrame(root, text="Drone Movement",width=200)
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

        droneDownBut = tk.Button(droneVerticalMovement, text="Down", width=10,bd=2, cursor="exchange", command = lambda: self.droneDown())
        droneDownBut.grid(row=1, column=0, columnspan=1) 

        #Rotational Movement
        droneRotationalMovement = tk.LabelFrame(droneMovement, text="Rotational Movement",width=200)
        droneRotationalMovement.grid(row=0, column=4, columnspan=2,rowspan=3,sticky="N")

        droneRotateLeftBut = tk.Button(droneRotationalMovement, text="Rotate Left", width=10,bd=2, cursor="exchange", command = lambda: self.droneRotateLeft())
        droneRotateLeftBut.grid(row=0, column=0, columnspan=1) 

        droneRotateRightBut = tk.Button(droneRotationalMovement, text="Rotate Right", width=10,bd=2, cursor="exchange", command = lambda: self.droneRotateRight())
        droneRotateRightBut.grid(row=0, column=1, columnspan=1)

        #Drone Movement by Pos
        dronePosMovement = tk.LabelFrame(root, text="Drone Movement(By Position)",width=200)
        dronePosMovement.grid(row=6, column=0, columnspan=10,sticky="W")
        
        #position X
        dronePosXLab = tk.Label(dronePosMovement, text="X:")
        dronePosXLab.grid(row=0, column=0, columnspan=1,sticky="W")

        vcmdPosX = (root.register(self.validateIsFloat),'%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        self.dronePosXInp = tk.StringVar(root, value="0.0") 
        dronePosXEnt = tk.Entry(dronePosMovement, validate = 'key', validatecommand = vcmdPosX, textvariable = self.dronePosXInp)
        #dronePosXEnt = tk.Entry(dronePosMovement, textvariable = self.dronePosX)
        dronePosXEnt.grid(row=0,column=1)
        
        #position Y
        dronePosYLab = tk.Label(dronePosMovement, text="Y:")
        dronePosYLab.grid(row=0, column=3, columnspan=1,sticky="W")

        vcmdPosY = (root.register(self.validateIsFloat),'%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        self.dronePosYInp = tk.StringVar(root, value="0.0")
        dronePosYEnt = tk.Entry(dronePosMovement, validate = 'key', validatecommand = vcmdPosY, textvariable = self.dronePosYInp)
        #dronePosYEnt = tk.Entry(dronePosMovement, textvariable = self.dronePosY)
        dronePosYEnt.grid(row=0,column=4)


        #position Z
        dronePosZLab = tk.Label(dronePosMovement, text="Z:")
        dronePosZLab.grid(row=0, column=5, columnspan=1,sticky="W")

        vcmdPosZ = (root.register(self.validateIsFloat),'%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        self.dronePosZInp = tk.StringVar(root, value="0.0")
        dronePosZEnt = tk.Entry(dronePosMovement, validate = 'key', validatecommand = vcmdPosZ, textvariable = self.dronePosZInp)
        dronePosZEnt.grid(row=0,column=6)
        
        #Rotation Z
        droneRotZLab = tk.Label(dronePosMovement, text="Rotation Z:")
        droneRotZLab.grid(row=0, column=7, columnspan=1,sticky="W")

        vcmdRotZ = (root.register(self.validateIsFloat),'%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        self.droneRotZInp = tk.StringVar(root, value="0.0")
        droneRotZEnt = tk.Entry(dronePosMovement, validate = 'key', validatecommand = vcmdRotZ, textvariable = self.droneRotZInp)
        droneRotZEnt.grid(row=0,column=8)

        self.droneNavPosXInfo = tk.StringVar(root, value="0.0")
        self.droneNavPosYInfo = tk.StringVar(root, value="0.0")
        self.droneNavPosZInfo = tk.StringVar(root, value="0.0")
        self.droneNavRotXInfo = tk.StringVar(root, value="0.0")
        self.droneNavRotYInfo = tk.StringVar(root, value="0.0")
        self.droneNavRotZInfo = tk.StringVar(root, value="0.0")

        rospy.loginfo("Drone Control Node: Setting GUI Finish")
        root.mainloop()

    def GUIinfoUpdate(self):
        self.droneNavPosXInfo.set()


    def start(self):
        #ROS Body
        while not rospy.is_shutdown():
            # rospy.spin() simply keeps python from exiting until this node is stopped

            rospy.loginfo("Drone Control Node: System Up")
            rospy.spin()


if __name__ == '__main__':
    my_node = drone_control_node()
    #start GUI
    GUIbody = threading.Thread(target = my_node.gui,daemon = True)
    GUIbody.start()
    #my_node.gui()

    #Start ROS part
    my_node.start()



        
