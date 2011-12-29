#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <arduino_hardware.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
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
// Redefine PROGMEM to avoid the warning: only initialized variables can be
// placed into program memory area which appears when compiling from the console
#include <avr/pgmspace.h>
#ifdef PROGMEM
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))
#endif

#include <ks0108.h>
#include "SystemFont5x7.h"

#define LCD_DEBUG_MSG_LEFT(...) \
    sprintf(g_dbg_text, __VA_ARGS__);\
    GLCD.CursorTo(0, g_dbg_line_left++ % 8);\
    GLCD.Puts(g_dbg_text)

#define LCD_DEBUG_MSG_RIGHT(...) \
    sprintf(g_dbg_text, __VA_ARGS__);\
    GLCD.CursorTo(12, g_dbg_line_right++ % 8);\
    GLCD.Puts(g_dbg_text)

#else  // LCD_DEBUG

#define LCD_DEBUG_MSG_LEFT(...)
#define LCD_DEBUG_MSG_RIGHT(...)

#endif  // LCD_DEBUG

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
struct PinConfig{
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

// True if the server works in continuous mode (send back values each cycle)
int g_continuous_mode = false;

enum ConfigCommand { REQUEST_CONFIG=0x00,
                     SET_FREQUENCY,
                     SET_CONTINUOUS_MODE,
                     SET_FILTER_LAMBDA,
                     SET_TIMESTAMP,
                     SET_ANALOG_REF };

// Sampling frequency in Hz.
int g_sample_frequency = 100;
// Maximal sample cycles between servo value refresh. The servos need
// refreshing every 40ms or so or they tend to forget their position and
// start to jitter. In order to have proper servo control make sure your
// sampling frequency is not lower than 25Hz.
int g_servo_refresh_cycle = (35 * g_sample_frequency) / 1000;

// The inner pin representation and status.
PinConfig g_pins[kPinCount];

// Indicate if all pins messages should be timed
bool g_timestamp = 0;
// Faked pin used to deliver time stamps.
const int kTimestampPin  = 0x8000;
// Time stamp mask
const int kTimestampMask = 0x7FFF;

// Indicator of the reference used for the A/D-Converter
byte g_analog_ref = DEFAULT;

// The lambda, forgetting factor for analog data filtering
// (~100 samples window)
float g_filter_lambda = 0.99;

// Maximal length of I2C message in bytes.
const int kMaxI2CResponseLen = 10;
// Maximal length of status response.
const int kMaxPoarkStatusLength = 250;

#ifdef LCD_DEBUG
// Buffer for debug text output to the display.
char g_dbg_text[20];
int g_dbg_line_left = 0;
int g_dbg_line_right = 0;
#endif  // LCD_DEBUG

// ROS Definitions
ArduinoHardware hardware;
ros::NodeHandle g_node_handle(&hardware);

// Output data buffer (+2 needed to include timestamp).
unsigned int g_ports_msg_out_data[2 * kPinCount + 2];
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

void ReadSamples();

inline void RequestStatusMsg(const char* msg) {
  // This test is to protect against strcpy-ing with the same source and
  // destination which is undefined.  If inlined, this should be optimized
  // away in most common cases.
  if (msg != g_poark_status_msg_out_data)
    strcpy(g_poark_status_msg_out_data, msg);
  g_need_poark_status_publish = true;
}

inline bool IsInputMode(PinConfig::PinMode mode) {
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
        filter_data = (1-g_filter_lambda)*reading + g_filter_lambda*filter_data;
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
void SetPinsState(const std_msgs::UInt8MultiArray& ports_msg_in) {
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
    if (g_pins[pin].pin_mode == PinConfig::SERVO)
      RequestStatusMsg(
          "{ error: \"Servo mode is not enabled.\"; error_code: 3; }");
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
    LCD_DEBUG_MSG_RIGHT("P%02d:%d=%d",
                        pin, g_pins[pin].pin_mode, g_pins[pin].state);
  }
}

// The callback for the set_pins_state message.
void SetPins(const std_msgs::UInt8MultiArray& pins_msg_in) {
  for (int i = 0;i < pins_msg_in.data_length/2;i++) {
    int pin = pins_msg_in.data[i*2 + 0];
    int state = pins_msg_in.data[i*2 + 1];
    if (!IsInputMode(g_pins[pin].pin_mode) &&
        g_pins[pin].pin_mode != PinConfig::NONE) {
      g_pins[pin].state = state;
      SetPin(pin, state);
    } else {
      state = 9;
      RequestStatusMsg(
          "{ error: \"Pin not in output mode.\"; error_code: 2; }");
    }
    LCD_DEBUG_MSG_RIGHT("S%02d=%d  ", pin, state);
  }
}

void RequestStatus(const std_msgs::Empty& empty_msg_in) {
  // Hold your breath for a huge ifdef orgy.
  // Note, this will overwrite any already queued messages!
  sprintf(g_poark_status_msg_out_data,
      "{\n  board_layout: \"%s\";\n  frequency: %d;"
      "\n  continuous mode: %d;\n  analog_ref: %d;"
      "\n  timestamp: %d;"
      "\n  filter_lambda_x_1000: %d;\n  with_servo: %c;"
      "\n  with_i2c: %c;\n  with_timer: %c;\n  lcd_debug: %c;\n}",
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      "mega_layout",
#else
      "mini_layout",
#endif  // ATmega[1280|2560]
      g_sample_frequency,
      g_continuous_mode,
      g_analog_ref,
      g_timestamp,
      static_cast<int>(g_filter_lambda * 1000),
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
  RequestStatusMsg(g_poark_status_msg_out_data);
  LCD_DEBUG_MSG_RIGHT("Status");
}

// The subscriber objects for set_pins_mode and set_pins_state.
ros::Subscriber<std_msgs::UInt8MultiArray> sub_set_pins_mode("set_pins_mode",
                                                             SetPinsState);
ros::Subscriber<std_msgs::UInt8MultiArray> sub_set_pins_state("set_pins_state",
                                                              SetPins);
ros::Subscriber<std_msgs::Empty> sub_request_config("request_poark_config",
                                                    RequestStatus);

#ifdef WITH_WIRE
// The publisher for i2c_response.
ros::Publisher pub_i2c_response("i2c_response", &g_i2c_msg_out);

// The callback for the i2c_io message.
void I2cIO(const std_msgs::UInt8MultiArray& i2c_msg_in) {
  int address = i2c_msg_in.data[0];
  int send_len = i2c_msg_in.data_length - 3;
  int receive_len =
      (i2c_msg_in.data[1] <= kMaxI2CResponseLen) ?
          i2c_msg_in.data[1] : kMaxI2CResponseLen;
  int token = i2c_msg_in.data[2];
  if (send_len > 0) {
    Wire.beginTransmission(address);
#if defined(ARDUINO) && ARDUINO >= 100
    Wire.write(&i2c_msg_in.data[3], send_len);
#else
    Wire.send(&i2c_msg_in.data[3], send_len);
#endif
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
#if defined(ARDUINO) && ARDUINO >= 100
        g_i2c_msg_out.data[g_i2c_msg_out.data_length] = Wire.read();
#else
        g_i2c_msg_out.data[g_i2c_msg_out.data_length] = Wire.receive();
#endif
    }
  }
  g_need_i2c_publish = true;
  LCD_DEBUG_MSG_RIGHT("I2C%d>%d<%d  ", address, send_len, receive_len);
}

ros::Subscriber<std_msgs::UInt8MultiArray> sub_i2c_io("i2c_io", I2cIO);

#endif  // WITH_WIRE

void SetConfig(const std_msgs::UInt16MultiArray& config_msg_in) {
  int index = 0;

  while (index < config_msg_in.data_length) {
    byte command = config_msg_in.data[index++];
    if (command != REQUEST_CONFIG && index == config_msg_in.data_length)
      return RequestStatusMsg("{ error: \"Not enough configure arguments.\";"
                              " error_code: 4; }");
    switch (command) {
      case REQUEST_CONFIG:
        RequestStatus(std_msgs::Empty());
        break;
      case SET_FREQUENCY:
        g_sample_frequency = config_msg_in.data[index++];
        g_servo_refresh_cycle = (35 * g_sample_frequency) / 1000;
#ifdef WITH_TIMER
        MsTimer2::set(1000 / g_sample_frequency, ReadSamples);
        MsTimer2::start();
#endif  // WITH_TIMER
        LCD_DEBUG_MSG_RIGHT("freq: %3d  ", g_sample_frequency);
        break;
      case SET_CONTINUOUS_MODE:
        g_continuous_mode = config_msg_in.data[index++] != 0;
        LCD_DEBUG_MSG_RIGHT("Cnt mode: %d", g_sample_frequency);
        break;
      case SET_FILTER_LAMBDA:
        g_filter_lambda = config_msg_in.data[index++]/1000.;
        LCD_DEBUG_MSG_RIGHT("Lambda: %d3",
                            static_cast<int>(1000*g_filter_lambda));
        break;
      case SET_TIMESTAMP:
        g_timestamp = (config_msg_in.data[index++] != 0);
        LCD_DEBUG_MSG_RIGHT("t stamp: %d ", static_cast<int>(g_timestamp));
        break;
      case SET_ANALOG_REF:
        g_analog_ref = config_msg_in.data[index++];
        analogReference(g_analog_ref);
        LCD_DEBUG_MSG_RIGHT("ARef: %d    ", static_cast<int>(g_analog_ref));
        break;
      default:
        RequestStatusMsg(
            "{ error: \"Unknown configure command.\"; error_code: 5; }");
        LCD_DEBUG_MSG_RIGHT("ERROR: conf");
        break;
    }
  }
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub_set_config("set_poark_config",
                                                           SetConfig);

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
    MsTimer2::set(1000 / g_sample_frequency, ReadSamples);
    MsTimer2::start();
    sampling_boost = false;
  }
#endif  // WITH_TIMER

#ifdef WITH_SERVO
  static byte servo_refresh = 0;
#endif  // WITH_SERVO

  unsigned int* msg_pointer = g_ports_msg_out.data;
  int out_pins_count = 0;
  if (g_timestamp) {
    unsigned long t = millis();
    msg_pointer[0] =
        kTimestampPin | static_cast<unsigned>((t>>16)&kTimestampMask);
    msg_pointer[1] = static_cast<unsigned>(t&0xFFFF);
    msg_pointer += 2;
  }
  for (int i = 0;i < kPinCount;i++) {
    if (IsInputMode(g_pins[i].pin_mode)) {
      int reading = GetPin(i);
      if (g_continuous_mode || reading != g_pins[i].reading) {
        msg_pointer[0] = i;
        msg_pointer[1] = reading;
        msg_pointer += 2;
        g_pins[i].reading = reading;
        out_pins_count++;
        LCD_DEBUG_MSG_LEFT("%02d:%4d", i, reading);
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
    g_ports_msg_out.data_length = msg_pointer - g_ports_msg_out.data;
    g_need_pin_state_publish = true;
  }
#ifdef WITH_SERVO
  if (!servo_refresh--)
    servo_refresh = g_servo_refresh_cycle;
#endif  // WITH_SERVO
#ifdef WITH_TIMER
  in_sampling = false;
#endif
}

// Arduino setup function. Called once for initialization.
void setup()
{
  // Define the output arrays.
  g_ports_msg_out.data_length = 2*kPinCount;
  g_ports_msg_out.data = g_ports_msg_out_data;
  g_i2c_msg_out.data_length = 255;
  g_i2c_msg_out.data = g_i2c_msg_out_data;
  g_poark_status_msg_out.data = g_poark_status_msg_out_data;

  // Digital and analog pin interface
  g_node_handle.advertise(pub_pin_state_changed);
  g_node_handle.subscribe(sub_set_pins_mode);
  g_node_handle.subscribe(sub_set_pins_state);
  // Status interface
  g_node_handle.advertise(pub_poark_status);
  g_node_handle.subscribe(sub_request_config);
  g_node_handle.subscribe(sub_set_config);

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
  MsTimer2::set(1000 / g_sample_frequency, ReadSamples);
  MsTimer2::start();
#endif  // WITH TIMER

  hardware.init();

  //initialize the LED output pin,
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, HIGH);
#ifdef LCD_DEBUG
  // ...and a display driver.
  GLCD.Init(NON_INVERTED);
  GLCD.ClearScreen();
  GLCD.SelectFont(System5x7);
  GLCD.CursorTo(0,0);
  GLCD.Puts_P(PSTR("Ready."));
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
  delay_time = 1000 / g_sample_frequency - delay_time;
  // If we needed too long to sample don't wait at all with the next cycle.
  if (delay_time > 0)
    delay(delay_time);
#endif  // WITH_TIMER
}
