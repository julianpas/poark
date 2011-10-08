#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"

#include <sstream>

const bool LOW = false;
const bool HIGH = true;

enum PinMode { OUT, IN, ANALOG, ANALOG_FILT, PWM_MODE, NONE=0xff };

const int kPinCount = 70;

const int kLed1RedPin = 8;
const int kBoardLedPin = 13;

const int kJoySwitchPin = 63;  // Pulls line down when pressed.
const int kJoyInterruptPin = 64;  // Active low interrupt input.
const int kJoyResetPin = 65;  // Active low reset output.

const int kJoyAddress = 0x40; // I2C Address of the joystick.

// Joystick state globals.
bool g_joystick_ready = false;
int g_x_pos = 0, g_y_pos = 0;
int g_x_center_pos = 0, g_y_center_pos = 0;
bool g_is_calibrated = false;
bool g_wait_for_callback = false;

// ROS objects. Initialized in main.
ros::NodeHandle *g_ros_node;
ros::Publisher g_i2c_pub;
ros::Publisher g_pins_state_pub;
ros::Publisher g_pins_pub;
ros::Rate *g_loop_rate;

// A callback for the /pins message from a Poark server.
// |msg| has the following layout:
// [ pin_id_1, pin_reading_1, pin_id_2, pin_reading_2, ...]
// For digital pins pin_reading_n will be either LOW=0 or HIGH = 1.
// For analog pins it will be a value between 0 and 1023.
void PinsCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{
  for (size_t i = 0;i < msg->data.size()/2;i++) {
    if (msg->data[i*2] == kJoyInterruptPin)
      g_joystick_ready = (msg->data[i*2 + 1] == 0);
  }
}

// Makes an I2C call to read a registry from the joystick.
// The i2c_io message has the following format:
// [ i2c_address, send_data_len, receive_data_len, token, data1, data2, ...].
// Where |i2c_address| is the address of the receiver of the message.
// both |send_data_len| and |receive_data_len| can be 0 in which case only
// one way communication is performed. |token| is a being sent back with the
// i2c_receive message beack from the poark_serve and can be anything the
// application needs.
void ReadJoyReg(int registry) {
  // Set the busy flag here and reset it when the callback has been fired.
  g_wait_for_callback = true;
  std_msgs::UInt8MultiArray msg2;
  msg2.data.clear();
  msg2.data.push_back(kJoyAddress);
  msg2.data.push_back(1);
  msg2.data.push_back(registry);
  msg2.data.push_back(registry);
  g_i2c_pub.publish(msg2);
  while (g_wait_for_callback)
    ros::spinOnce();
}

// Makes an I2C call to write to a registry on the joystick.
// See |ReadJoyReg| for data format description.
void WriteJoyReg(int registry, int value) {
  // Set the busy flag here and reset it when the callback has been fired.
  g_wait_for_callback = true;
  std_msgs::UInt8MultiArray msg2;
  msg2.data.clear();
  msg2.data.push_back(kJoyAddress);
  msg2.data.push_back(0);
  msg2.data.push_back(0);
  msg2.data.push_back(registry);
  msg2.data.push_back(value);
  g_i2c_pub.publish(msg2);
  while (g_wait_for_callback)
    ros::spinOnce();
}

// Procedure to calibrate the joystick taken from the demokit app.
void CalibrateJoystick() {
  WriteJoyReg(0x2e, 0x86);
  WriteJoyReg(0x0f, 0x00);
  ReadJoyReg(0x11);
  for (int i = 0; i < 16; ++i) {
    // Normally we should wait for |while (!g_joystick_ready);| here.
    ReadJoyReg(0x10);
    ReadJoyReg(0x11);
  }
  g_x_center_pos = -g_x_pos/16;
  g_y_center_pos = -g_y_pos/16;
  WriteJoyReg(0x12, 5 - g_x_center_pos);
  WriteJoyReg(0x13, -5 - g_x_center_pos);
  WriteJoyReg(0x14, 5 - g_y_center_pos);
  WriteJoyReg(0x15, -5 - g_y_center_pos);
  WriteJoyReg(0x0f, 0x04);
  g_is_calibrated = true;
}

// A callback for I2C data reads.
// Data format is as follows.
// [ i2c_address, token, data_len, data1, data2, ...]
// where |i2c_address| is the device that sent this data, |token| is the byte
// sent when issuing this request (see |ReadJoyReg|). |data_len| can be 0
// because the callback is called even if the receive_len has been set to zero.
void I2CCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
  g_wait_for_callback = false;
  switch(msg->data[1]) {
    case 0x00: // Write request has finished.
      return;
    case 0x0f: // Status read.
      ROS_INFO("I2C Status: %d", static_cast<int>(msg->data[2]));
      if ((msg->data[2] & 0xf0) == 0xf0)
        CalibrateJoystick();
      break;
    case 0x10: // X read out.
      if (g_is_calibrated)
        g_x_pos = msg->data[2] + g_x_center_pos;
      else
        g_x_pos += msg->data[2];
      break;
    case 0x11: // Y read out.
      if (g_is_calibrated)
        g_y_pos = msg->data[2] + g_y_center_pos;
      else
        g_y_pos += msg->data[2];
      ROS_INFO("I2C Coords: %3d %3d", g_x_pos, g_y_pos);
      break;
    default: // Unknown response.
      ROS_WARN("I2C received unknown response : Addr:%d Token:%d DataLen:%d",
               static_cast<int>(msg->data[0]),
               static_cast<int>(msg->data[1]),
               static_cast<int>(msg->data.size()));
  }
}

// Adds a pin definition for /set_pins_state message.
void AddPinDefinition(std_msgs::UInt8MultiArray* msg,
                     int pin,
                     PinMode mode,
                     int state) {
  msg->data.push_back(pin);
  msg->data.push_back(mode);
  msg->data.push_back(state);
}

// Adds a pin state for /set_pins message.
void AddPinState(std_msgs::UInt8MultiArray* msg, int pin, int state) {
  msg->data.push_back(pin);
  msg->data.push_back(state);
}

// Reset the joystick.
void ResetJoystick() {
  ROS_INFO("Sending reset signal to the Joystick.");
  std_msgs::UInt8MultiArray msg2;
  AddPinState(&msg2, kJoyResetPin, LOW);
  g_pins_pub.publish(msg2);
  ros::spinOnce();
  ros::Duration(0.001).sleep();
  msg2.data.clear();
  AddPinState(&msg2, kJoyResetPin, HIGH);
  g_pins_pub.publish(msg2);
  ros::spinOnce();
  ros::Duration(0.01).sleep();
  ReadJoyReg(0x0f);
}

// Reads the position of the joystick.
void ReadJoystick() {
  // Could have this here as well |while (!g_joystick_ready);|.
  ReadJoyReg(0x10);
  ReadJoyReg(0x11);
}

int main(int argc, char **argv)
{
  // Boot-up ROS.
  ros::init(argc, argv, "poark_client");
  // We are sending messages and spinning the ros loop from other functions too.
  g_ros_node = new ros::NodeHandle();
  g_loop_rate = new ros::Rate(100);
  g_pins_state_pub =
      g_ros_node->advertise<std_msgs::UInt8MultiArray>("set_pins_state", 1000);
  g_pins_pub =
      g_ros_node->advertise<std_msgs::UInt8MultiArray>("set_pins", 1000);
  g_i2c_pub =
      g_ros_node->advertise<std_msgs::UInt8MultiArray>("i2c_io", 1000);
  // Those don't have to be global.
  ros::Subscriber pins_sub =
      g_ros_node->subscribe("pins", 1000, PinsCallback);
  ros::Subscriber i2c_sub =
      g_ros_node->subscribe("i2c_response", 1000, I2CCallback);

  // Set up some pins in different modes.
  std_msgs::UInt8MultiArray msg;
  msg.data.clear();
  AddPinDefinition(&msg, kBoardLedPin, OUT, LOW);
  AddPinDefinition(&msg, kLed1RedPin, PWM_MODE, 255);
  AddPinDefinition(&msg, kJoySwitchPin, IN, HIGH);
  AddPinDefinition(&msg, kJoyInterruptPin, IN, HIGH);
  AddPinDefinition(&msg, kJoyResetPin, OUT, HIGH);
  ROS_INFO("Setting IO Pins.");
  g_pins_state_pub.publish(msg);
  ros::spinOnce();
  // Repeat the sending because ros-serial seems to eat our first message.
  ros::Duration(0.5).sleep();
  ROS_INFO("Setting IO Pins...again.");
  g_pins_state_pub.publish(msg);
  g_loop_rate->sleep();

  // Prepare the joystick.
  ResetJoystick();

  // Start the main loop.
  int count = 0;
  while (ros::ok())
  {
    if(count % 100 == 0) {
      std_msgs::UInt8MultiArray msg2;
      msg2.data.clear();
      AddPinState(&msg2, kLed1RedPin, (count % 200 == 0 ? 245 : 255));
      AddPinState(&msg2, kBoardLedPin, count % 200 == 0 ? HIGH : LOW);
      g_pins_pub.publish(msg2);
    }
    if (g_is_calibrated)
      ReadJoystick();
    ros::spinOnce();
    g_loop_rate->sleep();
    // A few seconds only for demo purposes should be enough.
    if (++count > 500)
      break;
  }
  // Clean up the state.
  msg.data.clear();
  AddPinDefinition(&msg, kBoardLedPin, NONE, LOW);
  AddPinDefinition(&msg, kLed1RedPin, PWM_MODE, 255);
  AddPinDefinition(&msg, kJoyInterruptPin, NONE, LOW);
  AddPinDefinition(&msg, kJoyResetPin, NONE, LOW);
  AddPinDefinition(&msg, kJoySwitchPin, NONE, LOW);
  ROS_INFO("Resetting all pins.");
  g_pins_state_pub.publish(msg);
  ros::spinOnce();
  return 0;
}

