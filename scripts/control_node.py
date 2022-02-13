#!/usr/bin/env python3

import rospy
import message_filters # To Achieve Multiple subscriber
#from std_msgs.msg import Float32
#from std_msgs.msg import Float64
#from std_msgs.msg import String
import threading
import tkinter as tk
import tkinter.font as tkfont
import time

class drone_control_node(object):
    def __init__(self):

        # Tells rospy the name of the node.
        # Anonymous = True makes sure the node has a unique name. Random
        # numbers are added to the end of the name.
        rospy.init_node("state_machine_node", anonymous=True, disable_signals=True)

        # Rate
        self.loop_rate = rospy.Rate(60)

        # Node is subscribing to the topic
        #self.vesc1_sub = message_filters.Subscriber('vesc1_speed', Float64)

        # Node is publishing to the topic
        #self.vesc1_pub = rospy.Publisher(vesc1_ns + '/commands/motor/speed', Float64, queue_size=10)

        #setup variable
        self.mode="manual"
        self.dronePosX =0.0
        self.dronePosY =0.0
        self.dronePosZ =0.0
        self.droneRotZ =0.0



    def validateIsFloat(self, action, index, value_if_allowed, prior_value, text, validation_type, trigger_type, widget_name):
        if value_if_allowed:
            try:
                float(value_if_allowed)
                return True
            except ValueError:
                return False
        else:
            return False

    def autoMode(self):
        self.mode="auto"
        rospy.loginfo("Drone Control Node: Setting Control to Auto(?) Position....")

    def visionPosMode(self):
        self.mode="visionPos"
        rospy.loginfo("Drone Control Node: Setting Control to Vision Position....")

    def manualMode(self):
        self.mode="manual"
        rospy.loginfo("Drone Control Node: Setting Control to Manual....")

    def px4DisArmMode(self):
        #later do
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to Disarm....")

    def px4PosMode(self):
        #later do
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to Position Mode....")

    def px4OffBoardMode(self):
        #later do
        rospy.loginfo("Drone Control Node: Setting PX4 Mode to Offboard Mode")

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

        px4PosBut = tk.Button(px4Mode, text="Disarm", width=10,bd=2, cursor="exchange", command = lambda: self.px4DisArmMode())
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
        dronePosXEnt = tk.Entry(dronePosMovement, validate = 'key', validatecommand = vcmdPosX)
        #dronePosXEnt = tk.Entry(dronePosMovement, textvariable = self.dronePosX)
        dronePosXEnt.grid(row=0,column=1)
        
        #position Y
        dronePosYLab = tk.Label(dronePosMovement, text="Y:")
        dronePosYLab.grid(row=0, column=3, columnspan=1,sticky="W")

        vcmdPosY = (root.register(self.validateIsFloat),'%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        dronePosYEnt = tk.Entry(dronePosMovement, validate = 'key', validatecommand = vcmdPosY)
        #dronePosYEnt = tk.Entry(dronePosMovement, textvariable = self.dronePosY)
        dronePosYEnt.grid(row=0,column=4)

        #position Z
        dronePosZLab = tk.Label(dronePosMovement, text="Z:")
        dronePosZLab.grid(row=0, column=5, columnspan=1,sticky="W")
        vcmdPosZ = (root.register(self.validateIsFloat),'%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')
        dronePosZEnt = tk.Entry(dronePosMovement, validate = 'key', validatecommand = vcmdPosZ)
        dronePosZEnt.grid(row=0,column=6)

        #Rotation







        rospy.loginfo("Drone Control Node: Setting GUI Finish")
        root.mainloop()

    def start(self):
        while not rospy.is_shutdown():
            # rospy.spin() simply keeps python from exiting until this node is stopped
            #rospy.loginfo("Drone Control Node: System Up")
            rospy.spin()


if __name__ == '__main__':
    my_node = drone_control_node()
    #start GUI
    t = threading.Thread(target = my_node.gui,daemon = True)
    t.start()
    #my_node.gui()

    #Start ROS part
    my_node.start()



        
