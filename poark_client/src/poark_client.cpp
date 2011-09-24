#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"

#include <sstream>

const bool LOW = false;
const bool HIGH = true;

enum PinDirection { OUT, IN, ANALOG, NONE };

const int kPinCount = 70;

// A callback for the /pins message from a Poark server.
// |msg| has the following layout:
// [ pin_id_1, pin_reading_1, pin_id_2, pin_reading_2, ...]
// For digital pins pin_reading_n will be either LOW=0 or HIGH = 1.
// For analog pins it will be a value between 0 and 1023.
void PinsCallback(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{
  for (size_t i = 0;i < msg->data.size()/2;i++)
    ROS_INFO("Pin %d : %d", (int)msg->data[i*2], (int)msg->data[i*2 + 1]);
}

// Adds a pin definition for /set_pins_state message.
void AddPinDefinition(std_msgs::UInt8MultiArray* msg,
                     int pin,
                     PinDirection direction,
                     bool state) {
  msg->data.push_back(pin);
  msg->data.push_back(direction);
  msg->data.push_back(state);
}

// Adds a pin state for /set_pins message.
void AddPinState(std_msgs::UInt8MultiArray* msg, int pin, bool state) {
  msg->data.push_back(pin);
  msg->data.push_back(state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "poark_client");
  ros::NodeHandle n;
  ros::Publisher pins_state_pub =
      n.advertise<std_msgs::UInt8MultiArray>("set_pins_state", 1000);
  ros::Publisher pins_pub =
      n.advertise<std_msgs::UInt8MultiArray>("set_pins", 1000);
  ros::Subscriber sub = n.subscribe("pins", 1000, PinsCallback);
  ros::Rate loop_rate(100);

  // Set up some pins in different modes.
  std_msgs::UInt8MultiArray msg;
  msg.data.clear();
  AddPinDefinition(&msg, 13, OUT, LOW);
  AddPinDefinition(&msg, 8, OUT, LOW);
  AddPinDefinition(&msg, 54, ANALOG, LOW);
  ROS_INFO("Sending /set_pins_state msg.");
  pins_state_pub.publish(msg);
  // Repeat the sending because ros-serial seems to eat our first message.
  ros::spinOnce();
  for (int i = 0;i < 20;i++)
    loop_rate.sleep();
  ROS_INFO("Sending /set_pins_state msg.");
  pins_state_pub.publish(msg);
  ros::spinOnce();
  loop_rate.sleep();
  // Start the main loop.
  int count = 0;
  while (ros::ok())
  {
    if(count % 100 == 0) {
      std_msgs::UInt8MultiArray msg2;
      msg2.data.clear();
      AddPinState(&msg2, 8, count % 400 == 0 ? HIGH : LOW);
      AddPinState(&msg2, 13, count % 200 == 0 ? HIGH : LOW);
      ROS_INFO("Sending set_pins msg : %d.", count);
      pins_pub.publish(msg2);
    }
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    if (count > 220)
      break;
  }

  msg.data.clear();
  AddPinDefinition(&msg, 13, NONE, LOW);
  AddPinDefinition(&msg, 8, NONE, LOW);
  AddPinDefinition(&msg, 54, NONE, LOW);
  ROS_INFO("Sending /set_pins_state msg.");
  pins_state_pub.publish(msg);
  return 0;
}

