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

package org.ros.tutorials.poark;

import com.google.common.base.Preconditions;

import org.apache.commons.logging.Log;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.poark.Poark;
import org.ros.poark.PoarkClient;
import org.ros.poark.PinChangeListener;

/**
 * This is a simple rosjava {@link Node} to demonstrate using the Poark.
 * It assumes an external roscore is already running.
 *
 * @author pastarmovj@google.com (Julian Pastarmov)
 */
public class DemoKit implements NodeMain {

  private Node node;
  private Log log;
  private PoarkClient poarkClient;

  // The relays use A0 and A1 for control.
  private final byte kRelay1 = Poark.A0;
  private final byte kRelay2 = Poark.A1;
  // Light and temp sensors use A2 and A3.
  private final byte kLightSensor = Poark.A2;
  private final byte kTempSensor = Poark.A3;
  // The three buttons on the demo shield are connected to pins A6 - A8.
  private final byte kButton1 = Poark.A6;
  private final byte kButton2 = Poark.A7;
  private final byte kButton3 = Poark.A8;
  // The joystick is connected on pins A9-A11.
  private final byte kJoySwitch = Poark.A9;
  private final byte kJoyInterrupt = Poark.A10;
  private final byte kJoyReset = Poark.A11;
  // The three RGB leds are controlled from pins 2-4,5-7,8-10
  private final byte kLed1Red = 2;
  private final byte kLed1Green = 3;
  private final byte kLed1Blue = 4;
  private final byte kLed2Red = 5;
  private final byte kLed2Green = 6;
  private final byte kLed2Blue = 7;
  private final byte kLed3Red = 8;
  private final byte kLed3Green = 9;
  private final byte kLed3Blue = 10;
  // The three servo pins are 11-13.
  private final byte kServo1 = 11;
  private final byte kServo2 = 12;
  private final byte kServo3 = 13;
  // the led on the main Arduino board is also connected to pin 13.
  private final byte kLedPin = 13;

  private int servoPosition = 90;

  // Handles events from the pins connected to push buttons.
  private class ButtonHandler implements PinChangeListener {
    @Override
    public void onPinStateChange(byte pin, int newValue) {
      if (newValue == Poark.LOW) {
        switch(pin) {
          // Use the buttons to control the servo position.
          case kButton1:
            servoPosition -= 10;
            if (servoPosition < 0)
              servoPosition = 0;
            break;
          case kButton2:
            servoPosition = 90;
            break;
          case kButton3:
            servoPosition += 10;
            if (servoPosition > 170) // 180 already makes my servo freak out.
              servoPosition = 170;
            break;
        }
        log.info("Pushed button: \"" + pin + " position: " + servoPosition + "\"");
        // Blink a led.
        poarkClient.setPinsState(
            new byte[] {kLed2Red, kLed2Green, kLed2Blue},
            new byte[] {(byte)255, (byte)200, (byte)255});
        try {
          Thread.sleep(100);
        } catch (InterruptedException e) { }
        poarkClient.setPinsState(new byte[] {kLed2Red, kLed2Green, kLed2Blue},
                                  new byte[] {(byte)255, (byte)255, (byte)255});
      }
    }
  }

  // Handles events from pins connected to the light and temperature sensors.
  private class LightAndTempHandler implements PinChangeListener {
    @Override
    public void onPinStateChange(byte pin, int newValue) {
      if (pin == kLightSensor) {
        // Make the light go dimmer when there is weak ambient light. Sort of auto
        // brightness control.
        byte ledStrength = (byte)(newValue / 4);
        poarkClient.setPinsState(
            new byte[] {kLed1Red, kLed1Green, kLed1Blue},
            new byte[] {ledStrength, ledStrength, ledStrength});
      } else {
        // Make the other light go redder the hotter it is.
        byte ledStrength = (byte)(255 - newValue / 4);
        poarkClient.setPinState(kLed3Red, ledStrength);
      }
    }
  }

  @Override
  public void main(NodeConfiguration configuration) {
    Preconditions.checkState(node == null);
    Preconditions.checkNotNull(configuration);
    try {
      node = new DefaultNodeFactory().newNode("basic_poark_client", configuration);
      log = node.getLog();
      // Attach the client to the node and add some pin listeners.
      poarkClient = new PoarkClient(Poark.BoardLayout.MegaLayout, node);
      // Send one dummy message to flush the communication.
      poarkClient.setPinMode(kLedPin, Poark.NONE, Poark.LOW);
      Thread.sleep(1000);
      // Set up the buttons.
      poarkClient.setPinsMode(
          new byte[]{kButton1, kButton2, kButton3},
          new byte[]{Poark.IN, Poark.IN, Poark.IN},
          new byte[]{Poark.HIGH, Poark.HIGH, Poark.HIGH});
      poarkClient.addPinChangeListener(kButton1, new ButtonHandler());
      poarkClient.addPinChangeListener(kButton2, new ButtonHandler());
      poarkClient.addPinChangeListener(kButton3, new ButtonHandler());
      // Set up the leds.
      poarkClient.setPinsMode(
          new byte[]{kLed1Red, kLed1Green, kLed1Blue},
          new byte[]{Poark.PWM_MODE, Poark.PWM_MODE, Poark.PWM_MODE},
          new byte[]{(byte)255, (byte)255, (byte)255});
      poarkClient.setPinsMode(
          new byte[]{kLed2Red, kLed2Green, kLed2Blue},
          new byte[]{Poark.PWM_MODE, Poark.PWM_MODE, Poark.PWM_MODE},
          new byte[]{(byte)255, (byte)255, (byte)255});
      poarkClient.setPinsMode(
          new byte[]{kLed3Red, kLed3Green, kLed3Blue},
          new byte[]{Poark.PWM_MODE, Poark.PWM_MODE, Poark.PWM_MODE},
          new byte[]{(byte)255, (byte)255, (byte)255});
      // Set up the temp sensor in filtered mode.
      poarkClient.setPinMode(kTempSensor, Poark.ANALOG_FILTERED, (byte)0);
      //poarkClient.SetPinMode(kLightSensor, Poark.ANALOG_FILTERED, (byte)0);
      poarkClient.addPinChangeListener(kTempSensor, new LightAndTempHandler());
      poarkClient.addPinChangeListener(kLightSensor, new LightAndTempHandler());
      // Set up the servo.
      poarkClient.setPinMode(kServo1, Poark.SERVO, (byte)0);

      // Sample I2C transfer.
      //poarkClient.AddI2CResponseListener((byte)0x40, new I2CResponseListener() {
      //  @Override
      //  public void onI2CResponse(byte address, byte token, byte[] data) {
      //    log.info("I2C Got : " + token + " - " + data[0]);
      //  }
      //});
      //poarkClient.I2CTransfer((byte)0x40, (byte)123, new byte[] {(byte)0x0f}, 1);
      // Some I2C fun.
      Thread.sleep(1000);  // Dramatic pause...
      log.info("Let's start");
      while (true) {
        poarkClient.setPinState(kServo1, (byte)servoPosition);
        Thread.sleep(20);
      }
    } catch (Exception e) {
      if (node != null) {
        node.getLog().fatal(e);
      } else {
        e.printStackTrace();
      }
    }
  }

  @Override
  public void shutdown() {
    node.shutdown();
    node = null;
  }
}
