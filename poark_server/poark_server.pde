#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>

#define WITH_SERVO 1

#ifdef WITH_SERVO
#include <Servo.h>
#endif

#define LCD_DEBUG 1

#ifdef LCD_DEBUG
#include <ks0108.h>  // library header
#include "SystemFont5x7.h"   // system font
#endif
////////////////////////
// Debug definitions.
const int kLedPin = 13;

////////////////////////
// Defines a pin and stores its state.
typedef struct PinConfig{
  enum PinMode { OUT, IN, ANALOG, ANALOG_FILT, PWM_MODE, SERVO, NONE=0xff };
  PinMode pin_mode;
  int state;
  int reading;
  float filter_data;
#ifdef WITH_SERVO
  Servo servo;
#endif
};
// Number of pins to be controlled (70 on a Mega board)
const int kPinCount = 70;

// Sampling frequency in Hz.
const int kSampleFrequency = 20;

// The inner pin representation and status.
PinConfig g_pins[kPinCount];

// The lambda, forgetting factor for analog data filtering (~100 samples window)
const float kFilterLambda = 0.99;

#ifdef LCD_DEBUG
// Buffer for debug text output to the display.
char text[20];
int line_left = 0;
int line_right = 0;
#endif

// ROS Definitions
ros::NodeHandle nh;

// Output data buffer.
unsigned int ports_msg_out_data[2 * kPinCount];
std_msgs::UInt16MultiArray ports_msg_out;

bool IsInputMode(PinConfig::PinMode mode) {
  return (mode == PinConfig::IN ||
          mode == PinConfig::ANALOG ||
          mode == PinConfig::ANALOG_FILT);
}

int GetPin(int pin) {
  switch (g_pins[pin].pin_mode) {
    case PinConfig::ANALOG: {
      return analogRead(pin);
    }
    case PinConfig::ANALOG_FILT: {
      float& filter_data = g_pins[pin].filter_data;
      int reading = analogRead(pin);
      if (filter_data == -1.)
        filter_data = reading;
      else
        filter_data = (1-kFilterLambda)*reading + kFilterLambda*filter_data;
      return static_cast<int>(filter_data + 0.5);
    }
    default: {  // Digital input
      return digitalRead(pin);
    }
  }
}

void SetPin(int pin, int state) {
  switch (g_pins[pin].pin_mode) {
    case PinConfig::PWM_MODE:
      analogWrite(pin, g_pins[pin].state);
      break;
#ifdef WITH_SERVO
    case PinConfig::SERVO:
      g_pins[pin].servo.attach(pin);
      g_pins[pin].servo.write(constrain(g_pins[pin].state, 0, 179));
      break;
#endif WITH_SERVO
    default:  // Digital output
      digitalWrite(pin, g_pins[pin].state);
  }
}

// The callback for the set_pins_state message.
ROS_CALLBACK(SetPinsState, std_msgs::UInt8MultiArray, ports_msg_in)
  for (int i = 0;i < ports_msg_in.data_length/3;i++) {
    int pin = ports_msg_in.data[i*3 + 0];
    g_pins[pin].pin_mode =
        static_cast<PinConfig::PinMode>(ports_msg_in.data[i*3 + 1]);
#ifdef WITH_SERVO
    if (g_pins[pin].pin_mode != PinConfig::SERVO &&
        g_pins[pin].servo.attached())
      g_pins[pin].servo.detach();
    if (g_pins[pin].pin_mode == PinConfig::SERVO &&
        !g_pins[pin].servo.attached())
      g_pins[pin].servo.attach(pin);
#endif
    g_pins[pin].state = ports_msg_in.data[i*3 + 2];
    g_pins[pin].reading = ports_msg_in.data[i*3 + 2];
    if (g_pins[pin].pin_mode != PinConfig::NONE) {
      if (g_pins[pin].pin_mode == PinConfig::ANALOG ||
          g_pins[pin].pin_mode == PinConfig::ANALOG_FILT) {
        // Analog pins should be set in input mode with low state to
        // operate correctly as analog pins.
        g_pins[pin].state = LOW;
        g_pins[pin].reading = 0;
      }
      g_pins[pin].filter_data = -1.;
      pinMode(pin, IsInputMode(g_pins[pin].pin_mode) ? INPUT : OUTPUT);
      // We have to set the state for both new in and out pins.
      SetPin(pin, g_pins[pin].state);
    }
#ifdef LCD_DEBUG
    sprintf(text, "P%02d:%d=%d",
            pin, g_pins[pin].pin_mode, g_pins[pin].state);
    GLCD.CursorTo(12,line_right++ % 8);
    GLCD.Puts(text);
#endif
  }
}

// The callback for the set_pins message.
ROS_CALLBACK(SetPins, std_msgs::UInt8MultiArray, pins_msg_in)
  for (int i = 0;i < pins_msg_in.data_length/2;i++) {
    int pin = pins_msg_in.data[i*2 + 0];
    int state = pins_msg_in.data[i*2 + 1];
    if (!IsInputMode(g_pins[pin].pin_mode) &&
        g_pins[pin].pin_mode != PinConfig::NONE) {
      g_pins[pin].state = state;
      SetPin(pin, state);
    }
#ifdef LCD_DEBUG
    else state = 9;
    sprintf(text, "S%02d=%d  ", pin, state);
    GLCD.CursorTo(12,line_right++ % 8);
    GLCD.Puts(text);
#endif
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
    g_pins[i].pin_mode = PinConfig::NONE;
    g_pins[i].state = LOW;
  }
  //initialize the LED output pin,
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, HIGH);
#ifdef LCD_DEBUG
  // ...and a display driver.
  GLCD.Init(NON_INVERTED);
  GLCD.ClearScreen();
  GLCD.SelectFont(System5x7);
  GLCD.CursorTo(0,0);
  GLCD.Puts("Ready.");
#endif
}

// The main loop. the Arduino bootloader will call this over
// and over ad nauseam until we power it down or reset the board.
void loop()
{
  int out_pins_count = 0;
  for (int i = 0;i < kPinCount;i++) {
    if (IsInputMode(g_pins[i].pin_mode)) {
      int reading = GetPin(i);
      if (reading != g_pins[i].reading) {
        ports_msg_out.data[out_pins_count * 2 + 0] = i;
        ports_msg_out.data[out_pins_count * 2 + 1] = reading;
        g_pins[i].reading = reading;
        out_pins_count++;
#ifdef LCD_DEBUG
        sprintf(text,"%02d:%4d", i, reading);
        GLCD.CursorTo(0,line_left++ % 8);
        GLCD.Puts(text);
#endif
      }
    }
  }
  if (out_pins_count > 0) {
    ports_msg_out.data_length = out_pins_count*2;
    int status = pub_pin_state_changed.publish(&ports_msg_out);
  }
  // Check for new messages and send all our output messages.
  nh.spinOnce();
  delay(1000 / kSampleFrequency);
}
