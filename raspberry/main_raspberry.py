#!/usr/bin/env python

import rospy
from cobots19.msg import LED
from cobots19.msg import Buttons
import RPi.GPIO as io

class main_raspberry():

    def __init__(self):
        # ROS node initialization
        rospy.init_node("Raspberry", anonymous=True)

        # Subscribers and Publishers:
        rospy.Subscriber("/led", LED, self.led_callback)
        self.buttons_publisher = rospy.Publisher("/buttons", Buttons, queue_size=10)

        # Message type initializers:
        self.led_msg = LED()
        self.buttons_msg = Buttons()
    
        # LED message value inits:
        self.led1 = False
        self.led2 = False
        self.led3 = False

        # Buttons message value inits:
        self.button1 = False
        self.button2 = False
        self.button3 = False
        self.button4 = False
        self.button5 = False

        self.buttons_msg.button1 = self.button1
        self.buttons_msg.button2 = self.button2
        self.buttons_msg.button3 = self.button3
        self.buttons_msg.button4 = self.button4
        self.buttons_msg.button5 = self.button5

        self.led_msg.led1 = self.led1
        self.led_msg.led2 = self.led2
        self.led_msg.led3 = self.led3
    
        # Publisher rates:
        #self.buttons_pub_rate = rospy.Rate(10)
       
        # Some time to assure initialization:
        rospy.sleep(2)

        # Board setup
        io.setmode(io.BOARD)

        #Setting up inputs for buttons
        io.setup(29, io.IN) #,pull_up_down = GPIO.PUD_DOWN) ???
        io.setup(31, io.IN)
        io.setup(33, io.IN)
        io.setup(35, io.IN)
        io.setup(37, io.IN)

        # Ground at pin 39

        #Setting upp outputs for LEDs
        io.setup(36, io.OUT)
        io.setup(38, io.OUT)
        io.setup(40, io.OUT)

    def main(self):
        tmp_but1 = self.button1
        tmp_but2 = self.button2
        tmp_but3 = self.button3
        tmp_but4 = self.button4
        tmp_but5 = self.button5

        while not rospy.is_shutdown():
            if io.input(29) == 0 :
                self.button1 = True
            else :
                self.button1 = False

            if io.input(31) == 0 :
                self.button2 = True
            else :
                self.button2 = False
            
            if io.input(33) == 0 :
                self.button3 = True
            else :
                self.button3 = False
            
            if io.input(35) == 0 :
                self.button4 = True
            else :
                self.button4 = False
            
            if io.input(37) == 0 :
                self.button5 = True
            else :
                self.button5 = False
            
            if not tmp_but1 == self.button1 or not tmp_but2 == self.button2 or not tmp_but3 == self.button3 or not tmp_but4 == self.button4 or not tmp_but5 == self.button5 :
                self.buttons_msg.button1 = self.button1
                self.buttons_msg.button2 = self.button2
                self.buttons_msg.button3 = self.button3
                self.buttons_msg.button4 = self.button4
                self.buttons_msg.button5 = self.button5

                self.button_pub(self.buttons_msg)

                tmp_but1 = self.button1
                tmp_but2 = self.button2
                tmp_but3 = self.button3
                tmp_but4 = self.button4
                tmp_but5 = self.button5

            
        rospy.spin()

    def led_callback(self, data):
       
        if data.led1:
            io.output(36,1)
        elif not data.led1:
            io.output(36,0)
        if data.led2:
            io.output(38,1)
        elif not data.led2:
            io.output(38,0)
        if data.led3:
            io.output(40,1)
        elif not data.led3:
            io.output(40,0)

    def button_pub(self, msg):
        self.buttons_publisher.publish(msg)
        

    