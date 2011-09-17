#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

#include <ks0108.h>  // library header
#include "SystemFont5x7.h"   // system font

////////////////////////
// Debug defintions.
const int kLedPin = 13;
enum PinDirection { OUT = 0, IN = 1, NONE = 2};
////////////////////////
// Defines a pin and stores its state.
typedef struct PinConfig{
  PinDirection pin_direction;
  bool state;
  bool reading;
};
// Number of pins to be controlled
const int kPinCount = 15;

// The inner pin representation and status.
PinConfig g_pins[kPinCount];

// Buffer for debug text output to the display.
char text[20];

// ROS Definitions
ros::NodeHandle nh;

// Output data buffer.
byte ports_msg_out_data[2 * kPinCount];
std_msgs::UInt8MultiArray ports_msg_out;

// The callback for the set_pins_state message.
ROS_CALLBACK(SetPinsState, std_msgs::UInt8MultiArray, ports_msg_in)
  for (int i = 0;i < ports_msg_in.data_length/3;i++) {
    g_pins[ports_msg_in.data[i*3 + 0]].pin_direction =
        (PinDirection)(ports_msg_in.data[i*3 + 1]);
    g_pins[ports_msg_in.data[i*3 + 0]].state =
        (ports_msg_in.data[i*3 + 2] ? true : false);
    g_pins[ports_msg_in.data[i*3 + 0]].reading =
        (ports_msg_in.data[i*3 + 2] ? true : false);
    if (g_pins[ports_msg_in.data[i*3 + 0]].pin_direction != NONE) {
      pinMode(ports_msg_in.data[i*3 + 0],
              g_pins[ports_msg_in.data[i*3 + 0]].pin_direction ?
                  INPUT : OUTPUT);
      // We have to set the state for both new in and out pins.
      digitalWrite(ports_msg_in.data[i*3 + 0],
                   g_pins[ports_msg_in.data[i*3 + 0]].state);
    }
    sprintf(text, "Pin %02d:%d[%d]",
            (int)ports_msg_in.data[i*3 + 0],
            (int)g_pins[ports_msg_in.data[i*3 + 0]].pin_direction,
            (int)g_pins[ports_msg_in.data[i*3 + 0]].state);
    GLCD.CursorTo(10,i);
    GLCD.Puts(text);
  }
}

// The communication primitives.
ros::Publisher pub_pin_state_changed("pins", &ports_msg_out);
ros::Subscriber sub_set_pin_state("set_pins_state",
                                  &ports_msg_in,
                                  &SetPinsState );

// Arduino setup function. Called once for initialization.
void setup()
{
  nh.initNode();

  // Define the output array.
  ports_msg_out.data_length = kPinCount*2;
  ports_msg_out.data = ports_msg_out_data;

  nh.advertise(pub_pin_state_changed);
  nh.subscribe(sub_set_pin_state);

  // Init all pins being neither in nor out.
  for (int i = 0;i < kPinCount;i++) {
    g_pins[i].pin_direction = NONE;
    g_pins[i].state = LOW;
  }
  //initialize the LED output pin,
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, HIGH);
  // ...and a display driver.
  GLCD.Init(NON_INVERTED);
  GLCD.ClearScreen();
  GLCD.SelectFont(System5x7);
  GLCD.CursorTo(0,0);
  GLCD.Puts("Ready.");
}

// The main loop. the Arduino bootloader will call this over
// and over ad nauseam until we power it down or reset the board.
void loop()
{
  int out_pins_count = 0;
  for (int i = 0;i < kPinCount;i++) {
    if (g_pins[i].pin_direction == IN) {
      bool reading = digitalRead(i);
      if (reading != g_pins[i].reading) {
        ports_msg_out.data[out_pins_count * 2 + 0] = i;
        ports_msg_out.data[out_pins_count * 2 + 1] = reading;
        g_pins[i].reading = reading;
        out_pins_count++;
        sprintf(text,"Pin %02d:%d  ", i, (int)reading);
        GLCD.CursorTo(0,out_pins_count-1);
        GLCD.Puts(text);
      }
    }
  }
  if (out_pins_count > 0) {
    ports_msg_out.data_length = out_pins_count*2;
    int status = pub_pin_state_changed.publish(&ports_msg_out);
    sprintf(text,"Status :%d", status);
    GLCD.CursorTo(0,out_pins_count);
    GLCD.Puts(text);
  }
  // Check for new messages and send all our output messages.
  nh.spinOnce();
  delay(50);
}
