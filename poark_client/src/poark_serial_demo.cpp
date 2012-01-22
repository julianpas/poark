#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"

#include <sstream>

enum ConfigCommand { REQUEST_CONFIG=0x00,
                     SET_FREQUENCY,
                     SET_CONTINUOUS_MODE,
                     SET_FILTER_LAMBDA,
                     SET_TIMESTAMP,
                     SET_ANALOG_REF,
                     SETUP_SERIAL };

enum BaudRate { BAUD0       = 0x0000,   // Hang up
                BAUD50      = 0x0001,   // 50 baud
                BAUD75      = 0x0002,   // 75 baud
                BAUD110     = 0x0003,   // 110 baud
                BAUD134     = 0x0004,   // 134.5 baud
                BAUD150     = 0x0005,   // 150 baud
                BAUD200     = 0x0006,   // 200 baud
                BAUD300     = 0x0007,   // 300 baud
                BAUD600     = 0x0008,   // 600 baud
                BAUD1200    = 0x0009,   // 1200 baud
                BAUD1800    = 0x000A,   // 1800 baud
                BAUD2400    = 0x000B,   // 2400 baud
                BAUD4800    = 0x000C,   // 4800 baud
                BAUD9600    = 0x000D,   // 9600 baud
                BAUD19200   = 0x000E,   // 19200 baud
                BAUD38400   = 0x000F,   // 38400 baud
                BAUD57600   = 0x1001,   // 57600 baud
                BAUD115200  = 0x1002,   // 115200 baud
                BAUD230400  = 0x1003,   // 230400 baud
                BAUD460800  = 0x1004,   // 460800 baud
                BAUD500000  = 0x1005,   // 500000 baud
                BAUD576000  = 0x1006,   // 576000 baud
                BAUD921600  = 0x1007,   // 921600 baud
                BAUD1000000 = 0x1008,   // 1000000 baud
                BAUD1152000 = 0x1009,   // 1152000 baud
                BAUD1500000 = 0x100A,   // 1500000 baud
                BAUD2000000 = 0x100B,   // 2000000 baud
                BAUD2500000 = 0x100C,   // 2500000 baud
                BAUD3000000 = 0x100D,   // 3000000 baud
                BAUD3500000 = 0x100E,   // 3500000 baud
                BAUD4000000 = 0x100F }; // 4000000 baud

// ROS objects. Initialized in main.
ros::Publisher g_poark_config;
bool g_serial_ready = true;
ros::Publisher g_serial1_pub;
ros::Publisher g_serial2_pub;


// Callback functions for the different serial messages.  It serves as an
// example how to receive data sent to a serial interface on the Arduino
// board.
template<int SERIAL_PORT>
void SerialCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
  std::ostringstream os;
  int data_len = msg->data.size();
  os << "Receive on Serial" << SERIAL_PORT << " (" << data_len << " B):\t";
  for (int i = 0; i < data_len; ++i) {

    int data = msg->data[i];
    os << data << "  ";
  }
  ROS_INFO("%s", os.str().c_str());
  g_serial_ready = true;
}

void AddSerialSetup(char port, BaudRate baud, std_msgs::UInt16MultiArray* msg) {
  msg->data.push_back(SETUP_SERIAL);
  msg->data.push_back(port);
  msg->data.push_back(baud);
}

int main(int argc, char **argv)
{
  // Boot-up ROS.
  ros::init(argc, argv, "poark_serial_demo");
  // We are sending messages and spinning the ros loop from other functions too.
  ros::NodeHandle ros_node;
  ros::Rate loop_rate(50);

  g_poark_config =
      ros_node.advertise<std_msgs::UInt16MultiArray>("set_poark_config", 1000);
  g_serial1_pub =
      ros_node.advertise<std_msgs::UInt8MultiArray>("serial1_send", 1000);
  ros::Subscriber serial1_sub =
      ros_node.subscribe("serial1_receive", 1000, SerialCallback<1>);
  ros::Subscriber serial2_sub =
      ros_node.subscribe("serial2_receive", 1000, SerialCallback<2>);

  std_msgs::UInt16MultiArray setup_msg;
  AddSerialSetup(1, BAUD9600, &setup_msg);  // Serial1, 9600 baud.
  AddSerialSetup(2, BAUD9600, &setup_msg);  // Serial2, 9600 baud.
  ROS_INFO("Sending set_poark_config message");
  g_poark_config.publish(setup_msg);
  ros::spinOnce();
  for (int i=0; i<20; ++i)
    loop_rate.sleep();

  ROS_INFO("Sending set_poark_config message");
  g_poark_config.publish(setup_msg);
  ros::spinOnce();
  loop_rate.sleep();

  // Start the main loop.
  for (int count=0; ros::ok() && count < 10;) {
    if (g_serial_ready) {
      g_serial_ready = false;
      std_msgs::UInt8MultiArray msg1;
      msg1.data.clear();
      msg1.data.push_back(count);
      msg1.data.push_back(2*count);
      msg1.data.push_back(3*count);
      ROS_INFO("   Send on Serial1 (%d B):\t%d  %d  %d",
               msg1.data.size(), msg1.data[0], msg1.data[1], msg1.data[2]);
      // Send 3 bytes on Serial1.
      g_serial1_pub.publish(msg1);
      ++count;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  for (int i=0; i<20; ++i) {  // Allow for last serial message to arrive.
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
