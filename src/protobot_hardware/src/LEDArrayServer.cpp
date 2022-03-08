/* MIT License

Copyright (c) [2022] [VIP Team RoSE]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#include "../include/LEDArrayServer.h"
#include "../include/LEDSettings.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "protobot_hardware_LED_array_server_node");
  ROS_INFO("Initializing LED_toggle_server");

  LEDSettings* ls_ptr = new LEDSettings;

  LEDArrayServer LEDArray_Server(ls_ptr);
  LEDArray_Server.LED_array.LEDInit(ls_ptr);

  LEDArray_Server.AdvertiseServerCheckCallback(&LEDArray_Server);

  ros::spin();

  LEDArray_Server.LED_array.LEDQuit();
  delete ls_ptr;

  return 0;
}

LEDArrayServer::LEDArrayServer(LEDSettings* ls_ptr) {
  GetYamlParams(ls_ptr);
}

bool LEDArrayServer::LEDCommandStatusCallback(
    protobot_hardware::LED_toggle::Request& req,
    protobot_hardware::LED_toggle::Response& res) {
  for (int8_t i = 0; i <= 3; i++) {
    if (req.LED_toggle == i) {
      CheckLEDToggle(req.LED_toggle);
      ROS_INFO("Received valid command from LED_toggle_client: "
               "Command: [%d]", req.LED_toggle);
      ROS_INFO("Toggling LED Array");
      LED_array.ToggleLEDArray(req.LED_toggle);
      res.reply = true;
      return true;
    }
  }
  ROS_INFO("Invalid command: [%d]", req.LED_toggle);
  return false;
}

void LEDArrayServer::AdvertiseServerCheckCallback(
    LEDArrayServer* LEDArray_Server) {
  ROS_INFO("Advertising LED_toggle_server");
  service = nh.advertiseService(
      "LED_toggle_server", &LEDArrayServer::LEDCommandStatusCallback,
      LEDArray_Server);
}

void LEDArrayServer::GetYamlParams(LEDSettings* ls_ptr) {
  nh.getParam("/LED_array_settings/serial_port", ls_ptr->serialPortAddr);
  nh.getParam("/LED_array_settings/baud_rate", ls_ptr->baudRate);
}

void LEDArrayServer::CheckLEDToggle(int8_t cmd) {
  switch (cmd) {
    case 0:
      ROS_INFO("Flashing Green Success");
      break;
    case 1:
      ROS_INFO("Autonomous Mode");
      break;
    case 2:
      ROS_INFO("Teleop Mode");
      break;
    case 3:
      ROS_INFO("LED Matrix Off");
      break;
    default:
      break;
  }
}
