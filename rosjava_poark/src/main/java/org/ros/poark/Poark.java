/*
 * Copyright (C) 2011 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.poark;

import org.ros.exception.ParameterNotFoundException;
import org.ros.message.std_msgs.UInt8MultiArray;

/**
 * This class provides some static methods and constants for basic Poark functionality.
 *
 * @author pastarmovj@google.com (Julian Pastarmov)
 */
public class Poark {
  // Board layouts
  public enum BoardLayout { MiniLayout, MegaLayout };

  // Pin digital state constants for maximal Arduino taste.
  public static final byte LOW = 0;
  public static final byte HIGH = 1;
  // Pin Modes as defined by the Poark server.
  public static final byte OUT = 0, IN = 1,                 // Digital I/O.
                           ANALOG = 2, ANALOG_FILTERED = 3, // Analog input.
                           PWM_MODE = 4,                    // PWM output.
                           SERVO = 5,                       // Servo output.
                           NONE = (byte)0xff;               // Pin not used.

  // Analog pins on Atmega 1280 or 2560 boards.
  public static final byte A0 = 54, A1 = 55, A2 = 56, A3 = 57, A4 = 58, A5 = 59, A6 = 60,
                           A7 = 61, A8 = 62, A9 = 63, A10 = 64, A11 = 65, A12 = 66,
                           A13 = 67, A14 = 68, A15 = 69;

  // Analog pins on Atmega 8 or 168 chips.
  public static final byte AI0 = 23, AI1 = 24, AI2 = 25, AI3 = 26, AI4 = 27, AI5 = 28;

  /**
   * Returns the pin number of analog pin depending on the board layout.
   *
   * @param analog_input the number of the analog pin as printed on the board.
   * @param layout Type of the board MiniLayout for Atmega 8,168 and MegaLayout for
   * Atmega 1280 or 2560.
   * @return The actual pin number as needed for pin I/O calls.
   * @throws ParameterNotFoundException if the board layout has no such pin.
   */
  public static byte getAPin(int analog_input,
                             BoardLayout layout) throws ParameterNotFoundException{
    if (layout == BoardLayout.MiniLayout) {
      if (analog_input < 0 || analog_input > 5)
        throw new ParameterNotFoundException("Analog pin number out of range [0..5].");
      return (byte)(AI0 + analog_input);
    } else {
      if (analog_input < 0 || analog_input > 15)
        throw new ParameterNotFoundException("Analog pin number out of range [0..15].");
      return (byte)(A0 + analog_input);
    }
  }

  /**
   * Creates an entry for a set_pins_state message. See the Poark's documentation for
   * more information regarding the set_pins_state message
   * {@linktourl http://code.google.com/p/poark/wiki/HowToUse#set_pins_state}.
   *
   * @param array Array to be extended with the new entry.
   * @param pin the pin to be set.
   * @param mode pin mode can be OUT, IN, ANALOG, ANALOG_FILTERED, PWM_MODE, SERVO or
   * NONE.
   * @param state - the initial pin state depends on the pin mode.
   */
  public static void createPinMode(UInt8MultiArray array,
                                   byte pin, byte mode, byte state) {
    byte[] new_data = new byte[array.data.length + 3];
    // Append the new pin state to the end.
    new_data[array.data.length] = pin;
    new_data[array.data.length + 1] = mode;
    new_data[array.data.length + 2] = state;
    System.arraycopy(array.data, 0, new_data, 0, array.data.length);
    array.data = new_data;
  }

  /**
   * Creates an entry for a set_pins message. See the Poark's documentation for
   * more information regarding the set_pins message
   * {@linktourl http://code.google.com/p/poark/wiki/HowToUse#set_pins}.
   *
   * @param array Array to be extended with the new entry.
   * @param pin the pin to be set.
   * @param state the new pin state depends on the pin mode.
   */
  public static void createPinState(UInt8MultiArray array, byte pin, byte state) {
    byte[] new_data = new byte[array.data.length + 2];
    // Append the new pin state to the end.
    new_data[array.data.length] = pin;
    new_data[array.data.length + 1] = state;
    System.arraycopy(array.data, 0, new_data, 0, array.data.length);
    array.data = new_data;
  }

  /**
   * Creates an I2C message buffer. See the Poark's documentation for
   * more information regarding the i2c_io message
   * {@linktourl http://code.google.com/p/poark/wiki/HowToUse#i2c_io}.
   * @param address The i2c slave address to talk to.
   * @param token User chosen token that will be sent back with the i2c_response message.
   * @param data The data to be sent to the i2c slave. Can be of zero length.
   * @param receive_length The number of bytes to wait for from the slave. Can be zero.
   * @return An array ready for sending in an i2c_io message.
   */
  public static UInt8MultiArray createI2CPackage(byte address, byte token,
                                                 byte[] data, int receive_length) {
    UInt8MultiArray i2c_io_data = new UInt8MultiArray();
    i2c_io_data.data = new byte[3 + data.length];
    i2c_io_data.data[0] = address;
    i2c_io_data.data[1] = (byte)receive_length;
    i2c_io_data.data[2] = token;
    System.arraycopy(data, 0, i2c_io_data.data, 3, data.length);
    return i2c_io_data;
  }
}
