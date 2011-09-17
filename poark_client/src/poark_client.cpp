#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"

#include <sstream>

const int kPinCount = 10;
/*
std_msgs::MultiArrayDimension ports_msg_in_dim[2];
unsigned char ports_msg_in_data[2 * kPinCount];
std_msgs::UInt8MultiArray ports_msg_in;

std_msgs::MultiArrayDimension ports_msg_out_dim[2];
unsigned char ports_msg_out_data[3 * kPinCount];
std_msgs::UInt8MultiArray ports_msg_out;

// Array dimention names.
unsigned char array_label_0[] = "pins";
unsigned char array_label_1[] = "pin_config";

void ConfigureMultiArray()
{
  ports_msg_in.layout.dim = ports_msg_in_dim;
  ports_msg_in.layout.dim[0].label = array_label_0;
  ports_msg_in.layout.dim[0].size = kPinCount;
  ports_msg_in.layout.dim[0].stride = kPinCount * 2;
  ports_msg_in.layout.dim[1].label = array_label_1;
  ports_msg_in.layout.dim[1].size = 2;
  ports_msg_in.layout.dim[1].stride = 2;
  ports_msg_in.layout.data_offset = 0;
  ports_msg_in.layout.dim_length = 2;
  ports_msg_in.data_length = kPinCount*2;
  ports_msg_in.data = ports_msg_in_data;
  
  ports_msg_out.layout.dim = ports_msg_out_dim;
  ports_msg_out.layout.dim[0].label = array_label_0;
  ports_msg_out.layout.dim[0].size = kPinCount;
  ports_msg_out.layout.dim[0].stride = kPinCount * 3;
  ports_msg_out.layout.dim[1].label = array_label_1;
  ports_msg_out.layout.dim[1].size = 3;
  ports_msg_out.layout.dim[1].stride = 3;
  ports_msg_out.layout.data_offset = 0;
  ports_msg_out.layout.dim_length = 2;
  ports_msg_out.data_length = kPinCount*3;
  ports_msg_out.data = ports_msg_out_data;
}
*/
void pinsCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
  for (int i = 0;i < msg->data.size()/2;i++)
    ROS_INFO("Pin%d : %d", (int)msg->data[i*2], (int)msg->data[i*2 + 1]);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "poark_client");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::UInt8MultiArray>("set_pins_state", 1000);
  ros::Subscriber sub = n.subscribe("pins", 1000, pinsCallback);
  ros::Rate loop_rate(10);
  
  int count = 0;
  while (ros::ok())
  {
    if(count % 100 == 0) {
      std_msgs::UInt8MultiArray msg;
      msg.data.clear();
      msg.data.push_back(13);
      msg.data.push_back(0);
      msg.data.push_back(count % 400 == 0);
      msg.data.push_back(9);
      msg.data.push_back(0);
      msg.data.push_back(count % 200 == 0);
      ROS_INFO("SENDING CMD %d", count);
      chatter_pub.publish(msg);
    }
    if(count % 500 == 0) {
      std_msgs::UInt8MultiArray msg;
      msg.data.clear();
      msg.data.push_back(8);
      msg.data.push_back(1);
      msg.data.push_back(0);
      ROS_INFO("SENDING RD CMD");
      chatter_pub.publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}

