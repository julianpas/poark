#!/usr/bin/env python
import roslib; roslib.load_manifest('poark_client')
import rospy
import tkColorChooser
from Tkinter import *
from std_msgs.msg import UInt8MultiArray, UInt16MultiArray

# This is a very very simple sample application to demonstrate the usage of the
# Poark server from a Python client. It uses only the basic pin functionallity
# to blink the LED on an arduino board and detect the state of a button.
#
# For more complex examples see the C++ primer in poark_client or the rosjava
# client which also has a neat high level interface.


# Some Poark constants:
# Pin digital states.
class PinState:
  HIGH = 1
  LOW = 0
# Pin modes as defined by the Arduino server.
class PinMode:
  OUT = 0
  IN = 1
  ANALOG = 2
  ANALOG_FILTERED = 3
  PWM_MODE = 4
  SERVO = 5
  NONE = 0xFF

# The led on the arduino board is connected to pin 13.
kLedPin = 13
# A6 on Mega board - first button on an Android demo shield.
kRedLed = 2
kGreenLed = 3
kBlueLed = 4
kRedLed2 = 5
kGreenLed2 = 6
kBlueLed2 = 7

# Callback for the pins message.
def pins_callback(data):
  rospy.loginfo("Button state changed. %d - %d"%(data.data[0], data.data[1]))

# The ROS messages we need.
pins_mode_pub = rospy.Publisher('set_pins_mode', UInt8MultiArray)
pins_state_pub = rospy.Publisher('set_pins_state', UInt8MultiArray)
pins_sub = rospy.Subscriber('pins', UInt16MultiArray, pins_callback)

# Sends out a set_pins_state message for a single pin.
def set_pin_state(pin, mode, initial_state):
  pins_state = UInt8MultiArray()
  pins_state.data = [pin, mode, initial_state]
  pins_mode_pub.publish(pins_state)

# Sends out a set_pins_state message for a single pin.
def set_pin(pin, state):
    pins = UInt8MultiArray()
    pins.data = [pin, state]
    pins_state_pub.publish(pins)

def set_leds(n, r,g,b):
    set_pin(kRedLed + n*3, r)
    set_pin(kGreenLed + n*3, g)
    set_pin(kBlueLed + n*3, b)
    rospy.sleep(0.05)


class Application(Frame):
  def say_hi(self):
    self.result = tkColorChooser.askcolor(color=self.result[1])
    set_leds(0, self.result[0][0], self.result[0][1], self.result[0][2]);

  def say_hi2(self):
    self.result = tkColorChooser.askcolor(color=self.result[1])
    set_leds(1, self.result[0][0], self.result[0][1], self.result[0][2]);

  def createWidgets(self):
    self.QUIT = Button(self)
    self.QUIT["text"] = "QUIT"
    self.QUIT["fg"]   = "red"
    self.QUIT["command"] =  self.quit
    self.QUIT.pack({"side": "left"})

    self.hi_there = Button(self)
    self.hi_there["text"] = "Color"
    self.hi_there["command"] = self.say_hi
    self.hi_there.pack({"side": "left"})

    self.hi_there2 = Button(self)
    self.hi_there2["text"] = "Color"
    self.hi_there2["command"] = self.say_hi2
    self.hi_there2.pack({"side": "left"})

  def __init__(self, master=None):
    Frame.__init__(self, master)
    self.pack()
    self.result = ((255,0,0),"red");
    self.createWidgets()


# Main loop that will turn the Arduino into XMas tree.
def init_poark():
  rospy.init_node('ss_enterprise')
  set_pin_state(kLedPin, PinMode.OUT, PinState.HIGH)
  rospy.sleep(1.0)
  set_pin_state(kLedPin, PinMode.OUT, PinState.HIGH)
  set_pin_state(kRedLed, PinMode.PWM_MODE, PinState.LOW)
  set_pin_state(kGreenLed, PinMode.PWM_MODE, PinState.LOW)
  set_pin_state(kBlueLed, PinMode.PWM_MODE, PinState.LOW)
  set_pin_state(kRedLed2, PinMode.PWM_MODE, PinState.LOW)
  set_pin_state(kGreenLed2, PinMode.PWM_MODE, PinState.LOW)
  set_pin_state(kBlueLed2, PinMode.PWM_MODE, PinState.LOW)
  r = g = b = 0;
  set_leds(0,r,g,b);
  set_leds(1,r,g,b);
#  for i in range(0,255,5):
#    set_leds(0,0,i)
#  while not rospy.is_shutdown():
#    for i in range(0,255,5):
#      set_leds(0,i,255)
#    for i in range(255,0,-5):
#      set_leds(0,255,i)
#    for i in range(0,255,5):
#      set_leds(i,255,0)
#    for i in range(255,0,-5):
#      set_leds(255,i,0)
#    for i in range(0,255,5):
#      set_leds(255,0,i)
#    for i in range(255,0,-5):
#      set_leds(i,0,255)

if __name__ == '__main__':
  try:
    init_poark()
    root = Tk()
    app = Application(master=root)
    app.mainloop()
    root.destroy()
  except rospy.ROSInterruptException: pass
