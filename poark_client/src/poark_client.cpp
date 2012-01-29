#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"

#include <sstream>

const bool LOW = false;
const bool HIGH = true;

enum PinMode { OUT,
               IN,
               ANALOG,
               ANALOG_FILT,
               PWM_MODE,
               SERVO,
               INTERRUPT,
               NONE=0xff };

enum InterruptMode { INT_LOW = 0x00,
                     INT_RISING = 0x03,
                     INT_FALLING = 0x02,
                     INT_CHANGE = 0x01,
                     INT_NONE = 0xff };

enum ConfigCommand { REQUEST_CONFIG=0x00,
                     SET_FREQUENCY,
                     SET_CONTINUOUS_MODE,
                     SET_FILTER_LAMBDA,
                     SET_TIMESTAMP,
                     SET_ANALOG_REF,
                     ERROR=0xff };

const int kPinCount = 70;
const int kLedPin = 13;
const int kServoControlPin = 54;
const int kServoPin = 7;
const int kInt = 18;  // Interrupt5 on pin 18.

const int kRLED = 2;
const int kGLED = 3;
const int kBLED = 4;

const int kTimestampPin  = 0x8000;
const int kTimestampMask = 0x7FFF;

// Variables to control the update of the servo position.  Keep
// volatile as they are changed asynchronously in a call back
// function.
volatile int g_servo_angle = 90;
volatile bool g_update_servo_angle = true;

// Variable to control the indicator led by a button, it's volatile
// by the same reason as above.
volatile bool g_led_state = false;
volatile bool g_update_led_state = false;

// A callback for the /pins message from a Poark server.
// |msg| has the following layout:
// [ pin_id_1, pin_reading_1, pin_id_2, pin_reading_2, ...]
// For digital pins pin_reading_n will be either LOW=0 or HIGH = 1.
// For analog pins it will be a value between 0 and 1023.
void PinsCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{
  for (size_t i = 0;i < msg->data.size()/2;i++) {
    int pin = static_cast<int>(msg->data[i*2]);
    int value = static_cast<int>(msg->data[i*2 + 1]);
    if (pin&kTimestampPin) {
      unsigned long t =
          (static_cast<unsigned long>(pin&kTimestampMask) << 16) + value;
      ROS_INFO("Time: %ld", t);
      continue;
    }

    ROS_INFO("Pin %d : %d", pin, value);
    if (pin == kServoControlPin) {
      // The servo control should be an angle between 0 and 180 degrees.
      int angle = static_cast<int>(value*180/1024);
      g_servo_angle = angle;
      g_update_servo_angle = true;
    } else if (pin == kInt) {
      // Ignore multiple activations in the same loop as this is likely a result
      // of poor connections resulting in several transitions.  Proper H/W
      // countermeasures should be applied in real applications.
      g_led_state = !g_led_state;
      g_update_led_state = true;
    }
  }
}

// A callback for the poark server message "poark_status" which is
// used by the poark server to communicate errors and requested status
// information.
void StatusCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("POARK STATUS MSG: %s", msg->data.c_str());
}

// Adds a pin definition for /set_pins_mode message.
void AddPinDefinition(std_msgs::UInt8MultiArray* msg,
                     int pin,
                     PinMode mode,
                     int state) {
  msg->data.push_back(pin);
  msg->data.push_back(mode);
  msg->data.push_back(state);
}

// Adds a pin state for /set_pins_state message.
void AddPinState(std_msgs::UInt8MultiArray* msg, int pin, int state) {
  msg->data.push_back(pin);
  msg->data.push_back(state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "poark_client");
  ros::NodeHandle n;
  ros::Publisher pins_mode_pub =
      n.advertise<std_msgs::UInt8MultiArray>("set_pins_mode", 1000);
  ros::Publisher pins_state_pub =
      n.advertise<std_msgs::UInt8MultiArray>("set_pins_state", 1000);
  ros::Publisher request_poark_config =
      n.advertise<std_msgs::Empty>("request_poark_config", 1000);
  ros::Publisher set_poark_config =
      n.advertise<std_msgs::UInt16MultiArray>("set_poark_config", 1000);
  ros::Subscriber sub = n.subscribe("pins", 1000, PinsCallback);
  ros::Subscriber status_sub =
      n.subscribe("poark_status", 1000, StatusCallback);

  ros::Rate loop_rate(100);

  // Set up some pins in different modes.
  std_msgs::UInt8MultiArray msg;
  msg.data.clear();
  AddPinDefinition(&msg, kRLED, PWM_MODE, 0);
  AddPinDefinition(&msg, kGLED, PWM_MODE, 0);
  AddPinDefinition(&msg, kBLED, PWM_MODE, 0);
  AddPinDefinition(&msg, kServoControlPin, ANALOG, LOW);
  AddPinDefinition(&msg, kServoPin, SERVO, 90);
  AddPinDefinition(&msg, kInt, INTERRUPT, INT_RISING);
  AddPinDefinition(&msg, kLedPin, OUT, g_led_state);
  ROS_INFO("Sending /set_pins_mode msg.");
  pins_mode_pub.publish(msg);
  // Repeat the sending because ros-serial seems to eat our first message.
  ros::spinOnce();
  for (int i = 0;i < 20;i++)
    loop_rate.sleep();
  ROS_INFO("Sending /set_pins_mode msg.");
  pins_mode_pub.publish(msg);
  ros::spinOnce();
  for (int i = 0; i < 20; ++i)
    loop_rate.sleep();

  std_msgs::UInt16MultiArray msg16;
  msg16.data.clear();
  msg16.data.push_back(SET_FREQUENCY);
  msg16.data.push_back(100);
  msg16.data.push_back(SET_CONTINUOUS_MODE);
  msg16.data.push_back(true);
  msg16.data.push_back(SET_TIMESTAMP);
  msg16.data.push_back(true);
  msg16.data.push_back(SET_ANALOG_REF);
  msg16.data.push_back(3);  // INTERNAL2V56
  set_poark_config.publish(msg16);
  ROS_INFO("Sending set_poark_config msg.");
  ros::spinOnce();
  for (int i = 0; i < 20; ++i)
    loop_rate.sleep();

  request_poark_config.publish(std_msgs::Empty());
  ROS_INFO("Sending request_poark_config msg.");
  ros::spinOnce();
  for (int i = 0; i < 20; ++i)
    loop_rate.sleep();

  // Start the main loop.
  int count = 0;
  while (ros::ok())
  {
    if (count % 10 == 0) {
      std_msgs::UInt8MultiArray msg2;
      msg2.data.clear();
      AddPinState(&msg2, kRLED, (count/10) % 50 + 200);
      AddPinState(&msg2, kGLED, (count/10) % 50 + 200);
      AddPinState(&msg2, kBLED, (count/10) % 50 + 200);
      ROS_INFO("Sending set_pins_state msg : %d.", count);
      pins_state_pub.publish(msg2);
    }
    if (g_update_servo_angle) {
      std_msgs::UInt8MultiArray msg;
      msg.data.clear();
      AddPinState(&msg, kServoPin, g_servo_angle);
      pins_state_pub.publish(msg);
      g_update_servo_angle = false;
    }
    if (g_update_led_state) {
      std_msgs::UInt8MultiArray msg;
      msg.data.clear();
      AddPinState(&msg, kLedPin, g_led_state);
      pins_state_pub.publish(msg);
      g_update_led_state = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    if (count > 3000)
      break;
  }

  msg.data.clear();
  AddPinDefinition(&msg, kRLED, PWM_MODE, 255);
  AddPinDefinition(&msg, kGLED, PWM_MODE, 255);
  AddPinDefinition(&msg, kBLED, PWM_MODE, 255);
  AddPinDefinition(&msg, kServoControlPin, NONE, LOW);
  AddPinDefinition(&msg, kServoPin, NONE, LOW);
  ROS_INFO("Sending /set_pins_mode msg.");
  pins_mode_pub.publish(msg);
  return 0;
}
