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

/**
 * This class declares the interface for a listener for the Poark pins message. See
 * @PoarkClient.AddPinchangeListener for more information.
 *
 * @author pastarmovj@google.com (Julian Pastarmov)
 */
public interface I2CResponseListener {

  /**
   * Called every time an I2C response was received.
   * @param address The address of the I2C communication.
   * @param token The token that was used for the i2c_io message.
   * @param data The data from the response (might be empty).
   */
  void onI2CResponse(byte address, byte token, byte[] data);
}