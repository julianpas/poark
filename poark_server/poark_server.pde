#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>

// Using LCD_DEBUG will slow down all operations due to the time needed to
// draw on the LCD. You should only use this for debugging and with lower
// sampling frequencies.
#define LCD_DEBUG 1
// If this flag is defined servo control will be enabled. This
// uses different interrupts for different pins on the board so
// make sure they won't interfere with any other functions you need.
#define WITH_SERVO 1
// The timer uses Timer 2 on the chip and will interfere with PWM on pins 9-11
// or servos connected to those pins.
#define WITH_TIMER 1
// The wire library is used for the I2C interface. If switched off all i2c
// related topics will not function.
#define WITH_WIRE 1

#ifdef LCD_DEBUG
#include <ks0108.h>
#include "SystemFont5x7.h"
#endif

#ifdef WITH_SERVO
#include <Servo.h>
#endif

#ifdef WITH_TIMER
#include <MsTimer2.h>
#endif

#ifdef WITH_WIRE
#include <Wire.h>
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
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
const int kPinCount = 70;
#else
const int kPinCount = 15;
#endif  // Atmega[1280|2560]

// Sampling frequency in Hz.
const int kSampleFrequency = 100;
// Maximal sample cycles between servo value refresh. The servos need
// refreshing every 40ms or so or they tend to forget their position and
// start to jitter. In order to have proper servo control make sure your
// sampling frequency is not lower than 25Hz.
const int kServoRefreshCycles = 35 * kSampleFrequency / 1000;

// The inner pin representation and status.
PinConfig g_pins[kPinCount];

// The lambda, forgetting factor for analog data filtering
// (~100 samples window)
const float kFilterLambda = 0.99;

// Maximal length of I2C message in bytes.
const int kMaxI2CResponseLen = 10;
// Maximal length of status response.
const int kMaxPoarkStatusLength = 150;

#ifdef LCD_DEBUG
// Buffer for debug text output to the display.
char g_dbg_text[20];
int g_dbg_line_left = 0;
int g_dbg_line_right = 0;
#endif  // LCD_DEBUG

// ROS Definitions
ros::NodeHandle g_node_handle;

// Output data buffer.
unsigned int g_ports_msg_out_data[2 * kPinCount];
byte g_i2c_msg_out_data[kMaxI2CResponseLen + 2];
char g_poark_status_msg_out_data[kMaxPoarkStatusLength + 1];
std_msgs::UInt16MultiArray g_ports_msg_out;
std_msgs::UInt8MultiArray g_i2c_msg_out;
std_msgs::String g_poark_status_msg_out;
bool g_need_i2c_publish = false;
// These variables will be read both from the main loop and the timer 
// interrupt therefore they shoud be volatile.
volatile bool g_need_poark_status_publish = false;
volatile bool g_need_pin_state_publish = false;
volatile bool g_publishing = false;

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
#endif  // WITH_SERVO
    default:  // Digital output
      digitalWrite(pin, g_pins[pin].state);
  }
}

// The communication primitives.
ros::Publisher pub_poark_status("poark_status", &g_poark_status_msg_out);
ros::Publisher pub_pin_state_changed("pins", &g_ports_msg_out);

// The callback for the set_pins_mode message.
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
#else
    if (g_pins[pin].pin_mode == PinConfig::SERVO) {
      g_need_poark_status_publish = true;
      strcpy(g_poark_status_msg_out_data,
             "{ error: \"Servo mode is not enabled.\"; error_code: 3; }");
    }
#endif  // WITH_SERVO
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
    sprintf(g_dbg_text, "P%02d:%d=%d",
            pin, g_pins[pin].pin_mode, g_pins[pin].state);
    GLCD.CursorTo(12,g_dbg_line_right++ % 8);
    GLCD.Puts(g_dbg_text);
#endif  // LCD_DEBUG
  }
}

// The callback for the set_pins_state message.
ROS_CALLBACK(SetPins, std_msgs::UInt8MultiArray, pins_msg_in)
  for (int i = 0;i < pins_msg_in.data_length/2;i++) {
    int pin = pins_msg_in.data[i*2 + 0];
    int state = pins_msg_in.data[i*2 + 1];
    if (!IsInputMode(g_pins[pin].pin_mode) &&
        g_pins[pin].pin_mode != PinConfig::NONE) {
      g_pins[pin].state = state;
      SetPin(pin, state);
    } else {
      state = 9;
      strcpy(g_poark_status_msg_out_data,
             "{ error: \"Pin not in output mode.\"; error_code: 2; }");
      g_need_poark_status_publish = true;
    }
#ifdef LCD_DEBUG
    sprintf(g_dbg_text, "S%02d=%d  ", pin, state);
    GLCD.CursorTo(12,g_dbg_line_right++ % 8);
    GLCD.Puts(g_dbg_text);
#endif  // LCD_DEBUG
  }
}

ROS_CALLBACK(RequestStatus, std_msgs::Empty, emtry_msg_in)
  // Hold your breath for a huge ifdef orgy.
  sprintf(g_poark_status_msg_out_data,
      "{\n  board_layout: \"%s\";\n  frequency: %d;"
      "\n  filter_lambda_x_1000: %d;\n  with_servo: %c;"
      "\n  with_i2c: %c;\n  with_timer: %c;\n  lcd_debug: %c;\n}",
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      "mega_layout",
#else
      "mini_layout",
#endif  // ATmega[1280|2560]
      kSampleFrequency,
      static_cast<int>(kFilterLambda * 1000),
#ifdef WITH_SERVO
      '1',
#else
      '0',
#endif  // WITH_SERVO
#ifdef WITH_WIRE
      '1',
#else
      '0',
#endif  // WITH_WIRE
#ifdef WITH_TIMER
      '1',
#else
      '0',
#endif  // WITH_TIMER
#ifdef LCD_DEBUG
      '1');
#else
      '0');
#endif  // LCD_DEBUG
  g_need_poark_status_publish = true;
#ifdef LCD_DEBUG
  sprintf(g_dbg_text, "Status");
  GLCD.CursorTo(12,g_dbg_line_right++ % 8);
  GLCD.Puts(g_dbg_text);
#endif
}

// The subscriber objects for set_pins_mode and set_pins_state.
ros::Subscriber sub_set_pins_mode("set_pins_mode",
                                   &ports_msg_in,
                                   &SetPinsState);
ros::Subscriber sub_set_pins_state("set_pins_state",
                             &pins_msg_in,
                             &SetPins);
ros::Subscriber sub_request_config("request_poark_config",
                                   &emtry_msg_in,
                                   &RequestStatus);

#ifdef WITH_WIRE
// The publisher for i2c_response.
ros::Publisher pub_i2c_response("i2c_response", &g_i2c_msg_out);

// The callback for the i2c_io message.
ROS_CALLBACK(I2cIO, std_msgs::UInt8MultiArray, i2c_msg_in)
  int address = i2c_msg_in.data[0];
  int send_len = i2c_msg_in.data_length - 3;
  int receive_len =
      (i2c_msg_in.data[1] <= kMaxI2CResponseLen) ?
          i2c_msg_in.data[1] : kMaxI2CResponseLen;
  int token = i2c_msg_in.data[2];
  if (send_len > 0) {
    Wire.beginTransmission(address);
    Wire.send(&i2c_msg_in.data[3], send_len);
    Wire.endTransmission();
  }
  g_i2c_msg_out.data_length = 2;
  g_i2c_msg_out.data[0] = address;
  g_i2c_msg_out.data[1] = token;
  if (receive_len > 0) {
    Wire.requestFrom(address, receive_len);
    for (int i = 0; i < receive_len; ++i, ++g_i2c_msg_out.data_length) {
      // TODO(pastarmovj): Investigate whether this issue with resending
      // the last byte is caused by the joystick or if it is an I2C
      // feature.
      while (Wire.available())
        g_i2c_msg_out.data[g_i2c_msg_out.data_length] = Wire.receive();
    }
  }
  g_need_i2c_publish = true;
#ifdef LCD_DEBUG
  sprintf(g_dbg_text, "I2C%d>%d<%d  ", address, send_len, receive_len);
  GLCD.CursorTo(12,g_dbg_line_right++ % 8);
  GLCD.Puts(g_dbg_text);
#endif  // LCD_DEBUG
}

ros::Subscriber sub_i2c_io("i2c_io",
                           &i2c_msg_in,
                           &I2cIO);

#endif  // WITH_WIRE

void ReadSamples() {
#ifdef WITH_TIMER
  static bool in_sampling = false;
  static bool sampling_boost = false;
  // Avoid reentrance.
  if (in_sampling || g_publishing) {
    // Increase sampling frequency temporarily to make sure sampling
    // will occur ASAP.
    if (!sampling_boost) {
      sampling_boost = true;
      MsTimer2::set(1, ReadSamples);
      MsTimer2::start();
    }
    return;
  }
  in_sampling = true;
  if (sampling_boost) {
    // If in sampling boost go back to normal mode.
    MsTimer2::set(1000 / kSampleFrequency, ReadSamples);
    MsTimer2::start();
    sampling_boost = false;
  }
#endif  // WITH_TIMER

#ifdef WITH_SERVO
  static byte servo_refresh = 0;
#endif  // WITH_SERVO

  int out_pins_count = 0;
  for (int i = 0;i < kPinCount;i++) {
    if (IsInputMode(g_pins[i].pin_mode)) {
      int reading = GetPin(i);
      if (reading != g_pins[i].reading) {
        g_ports_msg_out.data[out_pins_count * 2 + 0] = i;
        g_ports_msg_out.data[out_pins_count * 2 + 1] = reading;
        g_pins[i].reading = reading;
        out_pins_count++;
#ifdef LCD_DEBUG
        sprintf(g_dbg_text,"%02d:%4d", i, reading);
        GLCD.CursorTo(0,g_dbg_line_left++ % 8);
        GLCD.Puts(g_dbg_text);
#endif  // LCD_DEBUG
      }
#ifdef WITH_SERVO
    } else if (!servo_refresh && g_pins[i].pin_mode == PinConfig::SERVO) {
      // Servos must be refreshed every ~40ms or they tend to forget
      // where they are and start to jitter.
      SetPin(i, g_pins[i].state);
#endif  // WITH_SERVO
    }
  }
  // Anything changed?
  if (out_pins_count > 0) {
    g_ports_msg_out.data_length = out_pins_count*2;
    g_need_pin_state_publish = true;
  }
#ifdef WITH_SERVO
  if (!servo_refresh--)
    servo_refresh = kServoRefreshCycles;
#endif  // WITH_SERVO
#ifdef WITH_TIMER
  in_sampling = false;
#endif
}

// Arduino setup function. Called once for initialization.
void setup()
{
  g_node_handle.initNode();

  // Define the output arrays.
  g_ports_msg_out.data_length = kPinCount*4;
  g_ports_msg_out.data = g_ports_msg_out_data;
  g_i2c_msg_out.data_length = 255;
  g_i2c_msg_out.data = g_i2c_msg_out_data;
  g_poark_status_msg_out.data =
      reinterpret_cast<unsigned char*>(g_poark_status_msg_out_data);

  // Digital and analog pin interface
  g_node_handle.advertise(pub_pin_state_changed);
  g_node_handle.subscribe(sub_set_pins_mode);
  g_node_handle.subscribe(sub_set_pins_state);
  // Status interface
  g_node_handle.advertise(pub_poark_status);
  g_node_handle.subscribe(sub_request_config);

  // Init all pins being neither in nor out.
  for (int i = 0;i < kPinCount;i++) {
    g_pins[i].pin_mode = PinConfig::NONE;
    g_pins[i].state = LOW;
  }

#ifdef WITH_WIRE
  // I2C interface
  g_node_handle.advertise(pub_i2c_response);
  g_node_handle.subscribe(sub_i2c_io);
  Wire.begin();
#endif  // WITH_WIRE

#ifdef WITH_TIMER
  // Initialize the timer interrupt.
  MsTimer2::set(1000 / kSampleFrequency, ReadSamples);
  MsTimer2::start();
#endif  // WITH TIMER

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
#endif  // WITH_ LCD_DEBUG
}

// The main loop. the Arduino bootloader will call this over
// and over ad nauseam until we power it down or reset the board.
void loop()
{
#ifndef WITH_TIMER
  // If we don't sample on interrupt we have to try to be precise with the delay.
  long delay_time = millis();
  ReadSamples();
#else
  g_publishing = true;
#endif  // WITH_TIMER
  // Check for new messages and send all our output messages.
#ifdef WITH_I2C
  if (g_need_i2c_publish) {
    g_need_i2c_publish = false;
    pub_i2c_response.publish(&g_i2c_msg_out);
  }
#endif  // WITH_I2C
  if (g_need_pin_state_publish) {
    g_need_pin_state_publish = false;
    pub_pin_state_changed.publish(&g_ports_msg_out);
  }
  if (g_need_poark_status_publish) {
    g_need_poark_status_publish = false;
    pub_poark_status.publish(&g_poark_status_msg_out);
  }
  g_node_handle.spinOnce();
#ifdef WITH_TIMER
  g_publishing = false;
  // We need to loop only a tad faster than the sampling loop and ~500Hz is the
  // upper meaningfull boundary.
  delay(3);
#else
  delay_time = millis() - delay_time;
  // In case of overflow just take the time from 0 to now.
  // It will be inaccurate but seldom enough.
  if (delay_time < 0)
    delay_time = millis();
  delay_time = 1000 / kSampleFrequency - delay_time;
  // If we needed too long to sample don't wait at all with the next cycle.
  if (delay_time > 0)
    delay(delay_time);
#endif  // WITH_TIMER
}
