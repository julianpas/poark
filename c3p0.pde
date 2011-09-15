#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

#include <ks0108.h>  // library header
#include "SystemFont5x7.h"   // system font

////////////////////////
// Debug defintions.
const int kLedPin = 13;
// Buffer for text output to the display.
char text[20];
////////////////////////
// Defines a pin and stores its state.
typedef struct PinConfig{
  bool is_input; // TODO(pastarmovj): This has to be tristate - IN,OUT,NONE.
  bool state;
  bool reading;
};
// Number of pins to be controlled
const int kPinCount = 10;

PinConfig g_pins[kPinCount];

// ROS Definitions
ros::NodeHandle nh;

std_msgs::UInt8MultiArray ports_msg_out;

// Array dimention names.
unsigned char array_label_0[] = "pins";
unsigned char array_label_1[] = "pin_config";

// The callback for the set_pins_state message.
ROS_CALLBACK(SetPinsState, std_msgs::UInt8MultiArray, ports_msg_in)
  for (int i = 0;i < ports_msg_in.data_length;i++) {
    g_pins[ports_msg_in.data[i*2 + 0]].is_input = 
        (ports_msg_in.data[i*2 + 1] ? true : false);
    g_pins[ports_msg_in.data[i*2 + 0]].state = 
        (ports_msg_in.data[i*2 + 2] ? true : false);
    g_pins[ports_msg_in.data[i*2 + 0]].reading = 
        (ports_msg_in.data[i*2 + 2] ? true : false);
    pinMode(ports_msg_in.data[i*2 + 0],
            g_pins[ports_msg_in.data[i*2 + 0]].is_input ? INPUT : OUTPUT);
    // We have to set the state for both new in and out pins.
    digitalWrite(ports_msg_in.data[i*2 + 0],
                 g_pins[ports_msg_in.data[i*2 + 0]].state);
  }
  digitalWrite(kLedPin, HIGH-digitalRead(kLedPin)); // For debug flip the led.
}

ros::Publisher pub_pin_state_changed("pins", &ports_msg_out);
ros::Subscriber sub_set_pin_state("set_pins_state", &ports_msg_in, &SetPinsState );

void setup()
{
  nh.initNode();

  // Define the output array.
  ports_msg_out.layout.dim = (std_msgs::MultiArrayDimension *)
      malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  ports_msg_out.layout.dim[0].label = array_label_0;
  ports_msg_out.layout.dim[0].size = kPinCount;
  ports_msg_out.layout.dim[0].stride = kPinCount * 2;
  ports_msg_out.layout.dim[1].label = array_label_1;
  ports_msg_out.layout.dim[1].size = 2;
  ports_msg_out.layout.dim[1].stride = 2;
  ports_msg_out.layout.data_offset = 0;
  ports_msg_out.layout.dim_length = 2;
  ports_msg_out.data_length = kPinCount*2;
  ports_msg_out.data = (byte *)malloc(sizeof(byte)*2*kPinCount);

  nh.advertise(pub_pin_state_changed);
  nh.subscribe(sub_set_pin_state);

  // Init all pins being output.
  // TODO(pastarmovj): Make them being initialized to NONE.
  for (int i = 0;i < kPinCount;i++) {
    g_pins[i].is_input = false;
    g_pins[i].state = LOW;
  }
  // For debug set pin 7 to input.
  g_pins[7].is_input = true;
  g_pins[7].state = LOW;
  pinMode(7, INPUT);
  //initialize an LED output pin.
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, HIGH);
  // and a display driver.  
  GLCD.Init(NON_INVERTED);   // initialise the library
  GLCD.ClearScreen();  
  GLCD.SelectFont(System5x7);       // select fixed width system font 
  GLCD.CursorTo(0,0);
  GLCD.Puts("Ready.");
}

void loop()
{
  int out_pins_count = 0;
  for (int i = 0;i < kPinCount;i++) {
    if (g_pins[i].is_input) {
      bool reading = digitalRead(i);
      if (reading != g_pins[i].reading) {
        ports_msg_out.data[out_pins_count * 2 + 0] = i;
        ports_msg_out.data[out_pins_count * 2 + 1] = reading;
        g_pins[i].reading = reading;
        out_pins_count++;
        sprintf(text,"Pin %2d : %d", i, (int)reading);
        GLCD.CursorTo(0,out_pins_count-1);
        GLCD.Puts(text);
      }
    }
  }
  if (out_pins_count > 0) {
    ports_msg_out.data_length = out_pins_count*2;
    ports_msg_out.layout.dim[0].size = out_pins_count;
    ports_msg_out.layout.dim[0].stride = out_pins_count * 2;
    pub_pin_state_changed.publish(&ports_msg_out);
    digitalWrite(kLedPin, HIGH-digitalRead(kLedPin)); // For debug flip the led.
  }
  
  nh.spinOnce();
  delay(50);
}
