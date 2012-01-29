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
// The serial code depends on Arduino 1.0+.  If WITH_SERIAL is left undefined,
// no serial code will be generated and the serial interface cannot be used.
#if defined(ARDUINO) && ARDUINO >= 100
#define WITH_SERIAL 1
#endif // Arduino 1.0+

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define ARDUINO_MEGA 1
#endif  // ATmega[1280|2560]

#ifdef ARDUINO_MEGA
const int kInterruptCount = 6;
const int kSerialCount = 4;
#else
const int kInterruptCount = 2;
const int kSerialCount = 1;
#endif  // ARDUINO_MEGA

#ifdef LCD_DEBUG
// Redefine PROGMEM to avoid the warning: only initialized variables can be
// placed into program memory area which appears when compiling from the console
#include <avr/pgmspace.h>
#ifdef PROGMEM
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))
#endif

#include <glcd.h>
#include <fonts/SystemFont5x7.h>

#define LCD_DEBUG_MSG_LEFT(...) \
    sprintf(g_dbg_text, __VA_ARGS__);\
    GLCD.CursorTo(0, g_dbg_line_left++ % 8);\
    GLCD.Puts(g_dbg_text)

#define LCD_DEBUG_MSG_RIGHT(...) \
    sprintf(g_dbg_text, __VA_ARGS__);\
    GLCD.CursorTo(12, g_dbg_line_right++ % 8);\
    GLCD.Puts(g_dbg_text)

// Buffer for debug text output to the display.
char g_dbg_text[20];
int g_dbg_line_left = 0;
int g_dbg_line_right = 0;

inline void InitLCDDebug() {
  GLCD.Init(NON_INVERTED);
  GLCD.ClearScreen();
  GLCD.SelectFont(System5x7);
  GLCD.CursorTo(0,0);
  GLCD.Puts_P(PSTR("Ready."));
}

#else  // LCD_DEBUG

#define LCD_DEBUG_MSG_LEFT(...)
#define LCD_DEBUG_MSG_RIGHT(...)

inline void InitLCDDebug() {}

#endif  // LCD_DEBUG

#ifdef WITH_TIMER
#include <MsTimer2.h>
#endif // WITH_TIMER

#ifdef WITH_SERVO
#include <Servo.h>
#endif

#ifdef WITH_WIRE
#include <Wire.h>
#endif // WITH_WIRE

////////////////////////
// Debug definitions.
const int kLedPin = 13;

enum InterruptMode { INT_LOW = 0x00,
                     INT_RISING = 0x03,
                     INT_FALLING = 0x02,
                     INT_CHANGE = 0x01,
                     INT_NONE = 0xff };

////////////////////////
// Defines a pin and stores its state.
struct PinConfig{
  enum PinMode { OUT,
                 IN,
                 ANALOG,
                 ANALOG_FILT,
                 PWM_MODE,
                 SERVO,
                 INTERRUPT,
                 NONE=0xff };
  PinMode pin_mode;
  int state;
  int reading;
  float filter_data;
#ifdef WITH_SERVO
  Servo servo;
#endif
};
// Number of pins to be controlled (70 on a Mega board)
#ifdef ARDUINO_MEGA
const int kPinCount = 70;
#else
const int kPinCount = 15;
#endif  // ARDUINO_MEGA

// True if the server works in continuous mode (send back values each cycle)
int g_continuous_mode = false;

enum ConfigCommand { REQUEST_CONFIG=0x00,
                     SET_FREQUENCY,
                     SET_CONTINUOUS_MODE,
                     SET_FILTER_LAMBDA,
                     SET_TIMESTAMP,
                     SET_ANALOG_REF,
                     SETUP_SERIAL };

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

// ROS Definitions
ArduinoHardware g_hardware;
ros::NodeHandle g_node_handle(&g_hardware);

// Output data buffer (+2 needed to include timestamp).
unsigned int g_ports_msg_out_data[2 * kPinCount + 2];
std_msgs::UInt16MultiArray g_ports_msg_out;
// These variables will be read both from the main loop and the timer
// interrupt therefore they shoud be volatile.
volatile bool g_need_pin_state_publish = false;
volatile bool g_publishing = false;

void ReadSamples();
inline void RequestStatusMsg(const char* msg);

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
ros::Publisher pub_pin_state_changed("pins", &g_ports_msg_out);

volatile unsigned char g_interrupt_count[2][kInterruptCount] = { 0 };
volatile bool g_interrupt_count_buffer = 0;

template <int INTERRUPT>
void InterruptFun() {
 ++g_interrupt_count[g_interrupt_count_buffer][INTERRUPT];
}

inline byte Pin2Interrupt(byte pin) {
  switch (pin) {
    case 2:  // int 0  0x02
    case 3:  // int 1  0x03
      return pin & 0x01;
#ifdef ARDUINO_MEGA
    case 21: // int 2  0x15
    case 20: // int 3  0x14
    case 19: // int 4  0x13
    case 18: // int 5  0x12
      return pin - 13;
#endif  // ARDUINO_MEGA
    default:
      return 0;  // How report error
  }
}

inline byte Interrupt2Pin(byte interrupt) {
  static byte interrupt2Pin[kInterruptCount] =
#ifdef ARDUINO_MEGA
      { 2, 3, 21, 20, 19, 18 };
#else
      { 2, 3 };
#endif
  return interrupt2Pin[interrupt%kInterruptCount];
}

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
    if (g_pins[pin].pin_mode == PinConfig::INTERRUPT) {
      byte interrupt_mode = ports_msg_in.data[i*3 + 2];
      byte interrupt = Pin2Interrupt(pin);
      if (interrupt_mode == INT_NONE) {
        detachInterrupt(interrupt);
      } else {
        void (*interrupt_fun[kInterruptCount])() = {
          &InterruptFun<0>,
          &InterruptFun<1>,
#ifdef ARDUINO_MEGA
          &InterruptFun<2>,
          &InterruptFun<3>,
          &InterruptFun<4>,
          &InterruptFun<5>
#endif  //ARDUINO_MEGA
        };
        attachInterrupt(interrupt, interrupt_fun[interrupt], interrupt_mode);
      }
    }
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

// The subscriber objects for set_pins_mode and set_pins_state.
ros::Subscriber<std_msgs::UInt8MultiArray> sub_set_pins_mode("set_pins_mode",
                                                             SetPinsState);
ros::Subscriber<std_msgs::UInt8MultiArray> sub_set_pins_state("set_pins_state",
                                                              SetPins);

#ifdef WITH_WIRE
// Maximal length of I2C message in bytes.
const int kMaxI2CResponseLen = 10;
byte g_i2c_msg_out_data[kMaxI2CResponseLen + 2];
std_msgs::UInt8MultiArray g_i2c_msg_out;
bool g_need_i2c_publish = false;

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

inline void PublishI2CResponce() {
  if (g_need_i2c_publish) {
    g_need_i2c_publish = false;
    pub_i2c_response.publish(&g_i2c_msg_out);
  }
}

inline void InitI2CInterface() {
  // I2C interface
  g_i2c_msg_out.data_length = 255;
  g_i2c_msg_out.data = g_i2c_msg_out_data;

  g_node_handle.advertise(pub_i2c_response);
  g_node_handle.subscribe(sub_i2c_io);
  Wire.begin();
}
#else  // WITH_WIRE
inline void PublishI2CResponce() {}
inline void InitI2CInterface() {}
#endif  // WITH_WIRE

////////////////////////
// Serial interfaces

#if WITH_SERIAL
// Maximal length of Serial message in bytes.
const int kMaxSerialResponseLen = 10;

struct SerialConfig {
  HardwareSerial* serial;
  ros::Publisher* pub_receive;
  ros::Subscriber<std_msgs::UInt8MultiArray>* sub_send;
  bool active;
  bool need_publish;
  std_msgs::UInt8MultiArray msg_out;
  byte msg_out_data[kMaxSerialResponseLen + 2];
};

// Allow for easy initialization of the SerialConfig structure
// unfortunately the Arduino specifies the serial interfaces as
// Serial, Serial1, ... motivating the usage of a define.
#define INIT_SERIAL_CONFIG(C, N) \
    C.serial = &Serial##N; \
    C.pub_receive = &pub_serial##N##_receive; \
    C.sub_send = &sub_serial##N##_send; \
    C.active = false; \
    C.need_publish = false; \
    C.msg_out.data = C.msg_out_data; \
    C.msg_out.data_length = kMaxSerialResponseLen;

// Set up constants for serial baud rates, as defined in <termios.h> with
// s/B/BAUD/
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

long kBaudRateTable[32] = { 0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800,
                            2400, 4800, 9600, 19200, 38400, 57600, 115200,
                            230400, 460800, 500000, 576000, 921600, 1000000,
                            1152000, 150000, 2000000, 2500000, 3000000, 3500000,
                            4000000 };

SerialConfig g_serials[kSerialCount];

void SerialRecieve(int port_nr, SerialConfig* serial) {
  if (serial->active) {
    serial->msg_out.data_length = 0;
    while (serial->serial->available()) {
      serial->msg_out.data[serial->msg_out.data_length++] =
          serial->serial->read();
    }
    serial->need_publish = true;
    LCD_DEBUG_MSG_RIGHT("Ser%d<%d    ", port_nr, serial->msg_out.data_length);
  }
}

// serialEvent[1-3]?() are predefined callback functions in Arduino 1.0+
// that gets called as data is received on the serial interfaces.  This seems
// not to work as expected on Arduino versions prior to 1.0.
void serialEvent() {
  SerialRecieve(0, &g_serials[0]);
}

#ifdef ARDUINO_MEGA
void serialEvent1() {
  SerialRecieve(1, &g_serials[1]);
}

void serialEvent2() {
  SerialRecieve(2, &g_serials[2]);
}

void serialEvent3() {
  SerialRecieve(3, &g_serials[3]);
}
#endif  // ARDUINO_MEGA

// The callback for the serial_response.
ros::Publisher pub_serial_receive("serial_receive", &g_serials[0].msg_out);
#ifdef ARDUINO_MEGA
ros::Publisher pub_serial1_receive("serial1_receive", &g_serials[1].msg_out);
ros::Publisher pub_serial2_receive("serial2_receive", &g_serials[2].msg_out);
ros::Publisher pub_serial3_receive("serial3_receive", &g_serials[3].msg_out);
#endif  // ARDUINO_MEGA

// The callback for the serial_io message.  Make the template minimal
// to reduce (generated) code duplication.
void SerialSendHelper(const std_msgs::UInt8MultiArray& serial_msg_in,
                      int port_nr,
                      SerialConfig* serial) {
  int send_len = serial_msg_in.data_length;
  send_len = serial->serial->write(&serial_msg_in.data[0], send_len);
  serial->serial->flush();
  LCD_DEBUG_MSG_RIGHT("Ser%d>%d    ", port_nr, send_len);
}

template <int SERIAL_PORT>
void SerialSend(const std_msgs::UInt8MultiArray& serial_msg_in) {
  SerialSendHelper(serial_msg_in, SERIAL_PORT, &g_serials[SERIAL_PORT]);
}

ros::Subscriber<std_msgs::UInt8MultiArray> sub_serial_send("serial_send",
                                                           &SerialSend<0>);
#ifdef ARDUINO_MEGA
ros::Subscriber<std_msgs::UInt8MultiArray> sub_serial1_send("serial1_send",
                                                            &SerialSend<1>);
ros::Subscriber<std_msgs::UInt8MultiArray> sub_serial2_send("serial2_send",
                                                            &SerialSend<2>);
ros::Subscriber<std_msgs::UInt8MultiArray> sub_serial3_send("serial3_send",
                                                            &SerialSend<3>);
#endif  // ARDUINO_MEGA

inline void InitSerialInterface() {
  INIT_SERIAL_CONFIG(g_serials[0],);
#ifdef ARDUINO_MEGA
  INIT_SERIAL_CONFIG(g_serials[1], 1);
  INIT_SERIAL_CONFIG(g_serials[2], 2);
  INIT_SERIAL_CONFIG(g_serials[3], 3);
#endif  // ARDUINO_MEGA

  for (int i=0; i<kSerialCount; ++i) {
    g_node_handle.advertise(*g_serials[i].pub_receive);
    g_node_handle.subscribe(*g_serials[i].sub_send);
  }
}

inline void PublishSerialReceive() {
  for (int i=0; i<kSerialCount; ++i) {
    if (g_serials[i].need_publish) {
      g_serials[i].need_publish = false;
      g_serials[i].pub_receive->publish(&g_serials[i].msg_out);
    }
  }
}

void ConfigureSerial(const uint16_t* msg) {
  byte port = msg[0];
  // 32bit baud rate encoded with 16bit using kBaudRateTable.
  long baud = msg[1];
  if (port >= kSerialCount) {
    RequestStatusMsg(
        "{ error: \"Serial port unavailable.\"; error_code: 6; }");
  } else if (baud & ~0x100F) {
    RequestStatusMsg(
        "{ error: \"Serial port, unknown baud rate.\"; error_code: 7; }");
  } else {
    // Always close open serial connections before reinitialization.
    if (g_serials[port].active)
      g_serials[port].serial->end();
    if (baud > 0) {  // To deactivate a serial port use:  baud == 0
      g_serials[port].serial->begin(
          kBaudRateTable[(baud&0x0F) + ((baud>>12)&0x10)]);
      while (g_serials[port].serial->available())  // Empty UART cache.
        g_serials[port].serial->read();
      g_serials[port].active = true;
    } else {
      g_serials[port].active = false;
    }
  }
  LCD_DEBUG_MSG_LEFT("SP%1d: % 6d", port,
                     kBaudRateTable[(baud&0x0F)+((baud>>12)&0x10)]);
}
#else  // WITH_SERIAL
inline void InitSerialInterface() {}
inline void PublishSerialReceive() {}

void ConfigureSerial(const uint16_t*) {
  RequestStatusMsg(
      "{ error: \"Serial port unavailable.\"; error_code: 6; }");
}
#endif  // WITH_SERIAL

// Maximal length of status response.
const int kMaxPoarkStatusLength = 250;
char g_poark_status_msg_out_data[kMaxPoarkStatusLength + 1];
std_msgs::String g_poark_status_msg_out;
// This variable is read both from main loop and timer interrupt hence volatile.
volatile bool g_need_poark_status_publish = false;

inline void RequestStatusMsg(const char* msg) {
  // This test is to protect against strcpy-ing with the same source and
  // destination which is undefined.  If inlined, this should be optimized
  // away in most common cases.
  if (msg != g_poark_status_msg_out_data)
    strcpy(g_poark_status_msg_out_data, msg);
  g_need_poark_status_publish = true;
}

void RequestStatus(const std_msgs::Empty& empty_msg_in) {
  // Hold your breath for a huge ifdef orgy.
  // Note, this will overwrite any already queued messages!
  sprintf(g_poark_status_msg_out_data,
      "{\n  board_layout: \"%s\";\n  frequency: %d;"
      "\n  continuous mode: %d;\n  analog_ref: %d;"
      "\n  timestamp: %d;\n  serial_ports: %d"
      "\n  filter_lambda_x_1000: %d;\n  with_servo: %c;"
      "\n  with_i2c: %c;\n  with_timer: %c;\n  with_serial: %c;"
      "\n  lcd_debug: %c;\n}",
#ifdef ARDUINO_MEGA
      "mega_layout",
#else
      "mini_layout",
#endif  // ARDUINO_MEGA
      g_sample_frequency,
      g_continuous_mode,
      g_analog_ref,
      g_timestamp,
      kSerialCount,
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
#ifdef WITH_SERIAL
      '1',
#else
      '0',
#endif  // WITH_SERIAL
#ifdef LCD_DEBUG
      '1');
#else
      '0');
#endif  // LCD_DEBUG
  RequestStatusMsg(g_poark_status_msg_out_data);
  LCD_DEBUG_MSG_RIGHT("Status");
}

ros::Publisher pub_poark_status("poark_status", &g_poark_status_msg_out);
ros::Subscriber<std_msgs::Empty> sub_request_config("request_poark_config",
                                                    RequestStatus);

// Handle Poark configuration messages
inline bool CheckConfigDataLength(byte command, int len)
{
  if (command == REQUEST_CONFIG)
    return true;
  if (command == SETUP_SERIAL)
    return len>=2;
  else
    return len>=1;
}

void SetConfig(const std_msgs::UInt16MultiArray& config_msg_in) {
  int index = 0;

  while (index < config_msg_in.data_length) {
    byte command = config_msg_in.data[index++];
    if (CheckConfigDataLength(command, index - config_msg_in.data_length))
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
        LCD_DEBUG_MSG_RIGHT("Lambda: %3d",
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
      case SETUP_SERIAL: {
        ConfigureSerial(&config_msg_in.data[index]);
        index += 2;
        break;
      }
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

  // Have any watched interrupts triggered?
  g_interrupt_count_buffer = !g_interrupt_count_buffer;
  for (byte i=0; i<6; ++i) {
    if (byte count = g_interrupt_count[!g_interrupt_count_buffer][i]) {
      byte pin = Interrupt2Pin(i);
      msg_pointer[0] = pin;
      msg_pointer[1] = count;
      msg_pointer += 2;
      g_pins[pin].reading = count;
      ++out_pins_count;
      g_interrupt_count[!g_interrupt_count_buffer][i] = 0;
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

  InitI2CInterface();
  InitSerialInterface();

#ifdef WITH_TIMER
  // Initialize the timer interrupt.
  MsTimer2::set(1000 / g_sample_frequency, ReadSamples);
  MsTimer2::start();
#endif  // WITH TIMER

  g_hardware.init();

  //initialize the LED output pin,
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, HIGH);
  InitLCDDebug();
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
  PublishI2CResponce();
  PublishSerialReceive();
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
