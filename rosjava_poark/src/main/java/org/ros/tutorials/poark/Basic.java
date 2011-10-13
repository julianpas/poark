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
import org.ros.poark.PinChangeListener;
import org.ros.poark.Poark;
import org.ros.poark.PoarkClient;

/**
 * This is a simple rosjava {@link Node} to demonstrate using the Poark.
 * It assumes an external roscore is already running.
 *
 * @author pastarmovj@google.com (Julian Pastarmov)
 */
public class Basic implements NodeMain {

  private Node node;
  private PoarkClient poarkClient;
  // the led on the main Arduino board is connected to pin 13.
  private final byte kLedPin = 13;
  private final byte kButton1 = Poark.A6;

  @Override
  public void main(NodeConfiguration configuration) {
    Preconditions.checkState(node == null);
    Preconditions.checkNotNull(configuration);
    try {
      node = new DefaultNodeFactory().newNode("basic_poark_client", configuration);
      final Log log = node.getLog();
      // Atach the client to the node and add some pin listeners.
      poarkClient = new PoarkClient(Poark.BoardLayout.MegaLayout, node);
      poarkClient.addPinChangeListener(kButton1,
          new PinChangeListener() {
            @Override
            public void onPinStateChange(byte pin, int new_value) {
              log.info("Pushed the button : \"" + pin + " - " + new_value + "\"");
            }
          });
      poarkClient.setPinMode(kLedPin, Poark.OUT, Poark.HIGH);
      Thread.sleep(1000);
      // Setup the Poark I/O.
      poarkClient.setPinsMode(new byte[]{kLedPin, kButton1},
                               new byte[]{Poark.OUT, Poark.IN},
                               new byte[]{Poark.LOW, Poark.HIGH});
      // Start blinking the light.
      byte light = Poark.LOW;
      while (true) {
        light = (byte)(Poark.HIGH - light);
        poarkClient.setPinState(kLedPin, light);
        Thread.sleep(1000);
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
