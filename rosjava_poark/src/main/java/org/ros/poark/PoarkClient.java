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

import java.util.Collection;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Multimap;

import org.ros.exception.ParameterNotFoundException;
import org.ros.message.MessageListener;
import org.ros.message.std_msgs.UInt16MultiArray;
import org.ros.message.std_msgs.UInt8MultiArray;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.poark.Poark;
import org.ros.poark.PinChangeListener;

/**
 * This class wraps Poark messages in easy to use high level interface.
 *
 * @author pastarmovj@google.com (Julian Pastarmov)
 */
public class PoarkClient {

  private final int kMaxI2CDataLength = 10;

  private class PoarkPinsMessageListener implements MessageListener<UInt16MultiArray> {
    public Multimap<Byte, PinChangeListener> pinsListenerClients =
        HashMultimap.create();

    /**
     * Implements the pins listener and delegates messages to the registered
     * PinChangeLister objects.
     */
    @Override
    public void onNewMessage(UInt16MultiArray message) {
      Collection<PinChangeListener> globalListeners =
          pinsListenerClients.get((byte)-1);
      for (int i = 0;i < message.data.length / 2;++i) {
        // First notify all global listeners.
        for (PinChangeListener entry : globalListeners)
          entry.onPinStateChange((byte)message.data[i*2], message.data[i*2 + 1]);
        // Then all that are registered for this particular pin.
        for (PinChangeListener entry :
             pinsListenerClients.get((byte)message.data[i*2])) {
          entry.onPinStateChange((byte)message.data[i*2], message.data[i*2 + 1]);
        }
      }
    }
  }

  private class PoarkI2CMessageListener implements MessageListener<UInt8MultiArray> {
    public Multimap<Byte, I2CResponseListener> listenerClients =
      HashMultimap.create();

    /**
     * Implements the pins listener and delegates messages to the registered
     * PinChangeLister objects.
     */
    @Override
    public void onNewMessage(UInt8MultiArray message) {
      byte[] data = new byte[message.data.length - 2];
      System.arraycopy(message.data, 2, data, 0, data.length);
      for (I2CResponseListener entry : listenerClients.get((byte)-1))
        entry.onI2CResponse(message.data[0], message.data[1], data);
      // Then all that are registered for this particular address.
      for (I2CResponseListener entry : listenerClients.get(message.data[0]))
        entry.onI2CResponse(message.data[0], message.data[1], data);
    }
  }

  private Poark.BoardLayout layout;
  private Publisher<UInt8MultiArray> setPinStatePub;
  private Publisher<UInt8MultiArray> setPinsPub;
  private Publisher<UInt8MultiArray> i2cIoPub;
  private PoarkPinsMessageListener pinsListener = new PoarkPinsMessageListener();
  private PoarkI2CMessageListener i2cListener = new PoarkI2CMessageListener();
  /**
   * Contructs a PoarkClient instance.
   *
   * @param layout - The layout of the board to talk to.
   */
  public PoarkClient(Poark.BoardLayout layout, Node node) {
    this.layout = layout;
    setPinStatePub =
        node.newPublisher("set_pins_state", "std_msgs/UInt8MultiArray");
    setPinsPub =
        node.newPublisher("set_pins", "std_msgs/UInt8MultiArray");
    i2cIoPub =
      node.newPublisher("i2c_io", "std_msgs/UInt8MultiArray");
    node.newSubscriber("pins", "std_msgs/UInt16MultiArray", pinsListener);
    node.newSubscriber("i2c_response", "std_msgs/UInt8MultiArray", i2cListener);
  }

  /**
   * Adds a new pin listener for a specific pin or for all pins. Note that one listener
   * can be registered multiple times for different pins or that one listener can be used
   * for all pins at the same time.
   * {@linktourl http://code.google.com/p/poark/wiki/HowToUse#pins}
   * @param pin The pin number. If -1 is specified will listen to all pins.
   * @param listener The listener object.
   * @return True if the listener has been successfully added.
   */
  public boolean addPinChangeListener(byte pin, PinChangeListener listener) {
    return pinsListener.pinsListenerClients.put(pin, listener);
  }

  /**
   * Removes a listener.
   * @param pin The pin this listener was registered for.
   * @param listener The listener object to be removed.
   * @return True if the listener has been successfully removed.
   */
  public boolean removePinChangeListener(byte pin, PinChangeListener listener) {
    return pinsListener.pinsListenerClients.remove(pin, listener);
  }

  /**
   * Sends a set_pins_state message. See the Poark's documentation for
   * more information regarding the set_pins_state message
   * {@linktourl http://code.google.com/p/poark/wiki/HowToUse#set_pins_state}.
   *
   * @param pin - the pin to be set.
   * @param mode - pin mode can be OUT, IN, ANALOG, ANALOG_FILTERED, PWM_MODE, SERVO or
   * NONE.
   * @param state - the initial pin state depends on the pin mode.
   */
  public void setPinMode(byte pin, byte mode, byte initial_state) {
    UInt8MultiArray pinsModes = new UInt8MultiArray();
    Poark.createPinMode(pinsModes, pin, mode, initial_state);
    setPinStatePub.publish(pinsModes);
  }

  /**
   * The same as @SetPinMode but accept an array of pins to set with a single ROS message.
   * @param pins array of pin indexes to set
   * @param modes array of modes indexes to set
   * @param initialStates array of initial states to set
   * @throws IllegalArgumentException if the arrays have mismatching lengths.
   */
  public void setPinsMode(
      byte[] pins, byte[] modes, byte[] initialStates) throws IllegalArgumentException {
    if (pins.length != modes.length || pins.length != initialStates.length)
      throw new IllegalArgumentException();
    UInt8MultiArray pins_modes = new UInt8MultiArray();
    for (int i = 0;i < pins.length; ++i)
      Poark.createPinMode(pins_modes, pins[i], modes[i], initialStates[i]);
    setPinStatePub.publish(pins_modes);
  }

  /**
   * Creates an entry for a set_pins message. See the Poark's documentation for
   * more information regarding the set_pins message
   * {@linktourl http://code.google.com/p/poark/wiki/HowToUse#set_pins}.
   *
   * @param pin - the pin to be set.
   * @param state - the new pin state depends on the pin mode.
   */
  public void setPinState(byte pin, byte state) {
    UInt8MultiArray pinsStates = new UInt8MultiArray();
    Poark.createPinState(pinsStates, pin, state);
    setPinsPub.publish(pinsStates);
  }

  /**
   * Creates an entry for a set_pins message. See the Poark's documentation for
   * more information regarding the set_pins message
   * {@linktourl http://code.google.com/p/poark/wiki/HowToUse#set_pins}.
   *
   * @param pin - the pin to be set.
   * @param state - the new pin state depends on the pin mode.
   */
  public void setPinsState(byte[] pins, byte[] states) throws IllegalArgumentException {
    if (pins.length != states.length)
      throw new IllegalArgumentException();
    UInt8MultiArray pinsStates = new UInt8MultiArray();
    for (int i = 0;i < pins.length; ++i)
      Poark.createPinState(pinsStates, pins[i], states[i]);
    setPinsPub.publish(pinsStates);
  }

  /**
   * Initiates I2C transfer. Can be used for both sending and receiving data on the I2C
   * bus.
   * @param address The address of the slave device to talk to.
   * @param token A user defined token used to identify the corresponding i2c_repsonse.
   * @param data Up to (now) 10 bytes of data to transmit to the slave.
   * @param receiveLenght The amount of data to read from the slave device.
   * @throws IllegalArgumentException if the data to transmit is too long.
   */
  public void performI2CTransfer(byte address, byte token,
                                 byte[] data,
                                 int receiveLenght) throws IllegalArgumentException{
    if (data.length > kMaxI2CDataLength || receiveLenght > kMaxI2CDataLength)
      throw new IllegalArgumentException();
    UInt8MultiArray i2cIoData = Poark.createI2CPackage(
        address, token, data, receiveLenght);
    i2cIoPub.publish(i2cIoData);
  }

  /**
   * Adds a new i2c listener for a specific address or for all addresses. Note that one
   * listener can be registered multiple times for different addresses or that one
   * listener can be used for all addresses at the same time.
   * {@linktourl http://code.google.com/p/poark/wiki/HowToUse#i2c_response}
   * @param address The i2c slave address. If -1 is specified will listen to addresses.
   * @param listener The listener object.
   * @return True if the listener has been successfully added.
   */
  public boolean addI2CResponseListener(byte address, I2CResponseListener listener) {
    return i2cListener.listenerClients.put(address, listener);
  }

  /**
   * Removes a listener.
   * @param address The i2c slave address.
   * @param listener The listener object to be removed.
   * @return True if the listener has been successfully removed.
   */
  public boolean removeI2CResponseListener(
      byte address, I2CResponseListener listener) {
    return i2cListener.listenerClients.remove(address, listener);
  }

  /**
   * simplifies calls to @Poark.GetAPin.
   * @param analogInput - The analog pin number as printed on the board.
   * @return the real pin number as needed for the Poark.
   * @throws ParameterNotFoundException if the pin number is not valid for the current
   * layout.
   */
  public byte getAPin(int analogInput) throws ParameterNotFoundException{
    return Poark.getAPin(analogInput, layout);
  }
}
