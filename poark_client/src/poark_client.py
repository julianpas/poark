#!/usr/bin/env python
import roslib; roslib.load_manifest('poark_client')
import rospy
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
kButton = 60

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

# Main loop that will turn the Arduino into XMas tree.
def blinker():
  rospy.init_node('poark_blinker')
  set_pin_state(kLedPin, PinMode.OUT, PinState.HIGH)
  rospy.sleep(1.0)
  set_pin_state(kLedPin, PinMode.OUT, PinState.HIGH)
  set_pin_state(kButton, PinMode.IN, PinState.HIGH)
  toggle = PinState.HIGH;
  while not rospy.is_shutdown():
    toggle = PinState.HIGH - toggle
    set_pin(kLedPin, toggle)
    rospy.sleep(1.0)

if __name__ == '__main__':
  try:
    blinker()
  except rospy.ROSInterruptException: pass
