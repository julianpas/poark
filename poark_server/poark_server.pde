#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>

#include <ks0108.h>  // library header
#include "SystemFont5x7.h"   // system font

////////////////////////
// Debug defintions.
const int kLedPin = 13;

////////////////////////
// Defines a pin and stores its state.
typedef struct PinConfig{
  enum PinDirection { OUT, IN, ANALOG, NONE};
  PinDirection pin_direction;
  bool state;
  int reading;
};
// Number of pins to be controlled (70 on a Mega board)
const int kPinCount = 70;

// The inner pin representation and status.
PinConfig g_pins[kPinCount];

// Buffer for debug text output to the display.
char text[20];
int line_left = 0;
int line_right = 0;

// ROS Definitions
ros::NodeHandle nh;

// Output data buffer.
unsigned int ports_msg_out_data[2 * kPinCount];
std_msgs::UInt16MultiArray ports_msg_out;

// The callback for the set_pins_state message.
ROS_CALLBACK(SetPinsState, std_msgs::UInt8MultiArray, ports_msg_in)
  for (int i = 0;i < ports_msg_in.data_length/3;i++) {
    int pin = ports_msg_in.data[i*3 + 0];
    g_pins[pin].pin_direction =
        (PinConfig::PinDirection)(ports_msg_in.data[i*3 + 1]);
    g_pins[pin].state = (ports_msg_in.data[i*3 + 2] ? HIGH : LOW);
    g_pins[pin].reading = (ports_msg_in.data[i*3 + 2] ? HIGH : LOW);
    if (g_pins[pin].pin_direction != PinConfig::NONE) {
      if (g_pins[pin].pin_direction == PinConfig::ANALOG) {
        // Analog pins should be set in input mode with low state to
        // operate correctly as analog pins.
        g_pins[pin].state = LOW;
        g_pins[pin].reading = 0;
      }
      pinMode(pin, g_pins[pin].pin_direction ? INPUT : OUTPUT);
      // We have to set the state for both new in and out pins.
      digitalWrite(pin, g_pins[pin].state);
    }
    sprintf(text, "Pin %02d:%d[%d]",
            (int)ports_msg_in.data[i*3 + 0],
            (int)g_pins[ports_msg_in.data[i*3 + 0]].pin_direction,
            (int)g_pins[ports_msg_in.data[i*3 + 0]].state);
    GLCD.CursorTo(10,line_right++ % 7);
    GLCD.Puts(text);
  }
}

// The callback for the set_pins message.
ROS_CALLBACK(SetPins, std_msgs::UInt8MultiArray, pins_msg_in)
  for (int i = 0;i < pins_msg_in.data_length/2;i++) {
    int pin = pins_msg_in.data[i*2 + 0];
    int state = pins_msg_in.data[i*2 + 1];
    if (g_pins[pin].pin_direction == PinConfig::OUT) {
      g_pins[pin].state = state;
      digitalWrite(pin, state);
    }
    else state = 9;
    sprintf(text, "Set %02d=[%d]   ", pin, state);
    GLCD.CursorTo(10,line_right++ % 7);
    GLCD.Puts(text);
  }
}

// The communication primitives.
ros::Publisher pub_pin_state_changed("pins", &ports_msg_out);
ros::Subscriber sub_set_pins_state("set_pins_state",
                                   &ports_msg_in,
                                   &SetPinsState);
ros::Subscriber sub_set_pins("set_pins",
                             &pins_msg_in,
                             &SetPins);

// Arduino setup function. Called once for initialization.
void setup()
{
  nh.initNode();

  // Define the output array.
  ports_msg_out.data_length = kPinCount*4;
  ports_msg_out.data = ports_msg_out_data;

  nh.advertise(pub_pin_state_changed);
  nh.subscribe(sub_set_pins_state);
  nh.subscribe(sub_set_pins);

  // Init all pins being neither in nor out.
  for (int i = 0;i < kPinCount;i++) {
    g_pins[i].pin_direction = PinConfig::NONE;
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
    if (g_pins[i].pin_direction == PinConfig::IN) {
      int reading = digitalRead(i);
      if (reading != g_pins[i].reading) {
        ports_msg_out.data[out_pins_count * 2 + 0] = i;
        ports_msg_out.data[out_pins_count * 2 + 1] = reading;
        g_pins[i].reading = reading;
        out_pins_count++;
        sprintf(text,"Pin %02d:%d   ", i, (int)reading);
        GLCD.CursorTo(0,line_left++ % 7);
        GLCD.Puts(text);
      }
    } else if (g_pins[i].pin_direction == PinConfig::ANALOG) {
      int reading = analogRead(i);
      if (reading != g_pins[i].reading) {
        ports_msg_out.data[out_pins_count * 2 + 0] = i;
        ports_msg_out.data[out_pins_count * 2 + 1] = reading;
        g_pins[i].reading = reading;
        out_pins_count++;
        sprintf(text,"Pin %02d:%04d", i, (int)reading);
        GLCD.CursorTo(0,line_left++ % 7);
        GLCD.Puts(text);
      }
    }
  }
  if (out_pins_count > 0) {
    ports_msg_out.data_length = out_pins_count*2;
    int status = pub_pin_state_changed.publish(&ports_msg_out);
  }
  // Check for new messages and send all our output messages.
  nh.spinOnce();
  delay(50);
}
