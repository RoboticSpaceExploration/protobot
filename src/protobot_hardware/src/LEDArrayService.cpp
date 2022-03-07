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

#include "../include/LEDArrayService.h"
#include "../include/LEDSettings.h"

int main (int argc, char** argv) {
  ros::init(argc, argv, "LED_array_server");
  ROS_INFO("Initializing LED array toggle service");

  ros::NodeHandle nh;
  ros::Rate rate(1.0);
  ros::AsyncSpinner spinner(4);
  spinner.start();

  LEDSettings* ls_ptr = new LEDSettings;
  LEDArrayService LEDArray_Service(ls_ptr, &nh);
  LEDArray LED_array(ls_ptr);

  LED_array.LEDInit();

  while (ros::ok()) {
    LEDArray_Service.AdvertiseService(&LEDArray_Service, &nh);
    LEDArray_Service.SendCommandToHardware(ls_ptr, &LED_array);
    ros::spinOnce();
    rate.sleep();
  }

  LED_array.LEDQuit();
  delete ls_ptr;

  return 0;
}

LEDArrayService::LEDArrayService(LEDSettings* ls, ros::NodeHandle* nh) {
  GetYamlParams(ls, nh);
}

void LEDArrayService::GetYamlParams(LEDSettings* ls, ros::NodeHandle* nh) {
  nh->getParam("/LED_array_settings/serial_port", ls->serialPortAddr);
  nh->getParam("/LED_array_settings/baud_rate", ls->baudRate);
}

bool LEDArrayService::LEDCommandStatusCallback(
    protobot_hardware::LED_toggle::Request& req,
    protobot_hardware::LED_toggle::Response& res) {
  ROS_WARN("IS THIS CALLBACK WORKING?");
  for (int8_t i = 0; i < 3; i++) {
    if (req.LED_toggle == i) {
      ROS_WARN("Received LED Array Command: [%d]", req.LED_toggle);
      cmd = req.LED_toggle;
      res.reply = true;  // copy LED command to class
      reply = true;      // copy reply to class
      return true;
    }
  }

  return false;
}

void LEDArrayService::AdvertiseService(
    LEDArrayService* LEDArray_Service, ros::NodeHandle* nh) {
  ROS_INFO("Advertising LED array service");
  service = nh->advertiseService(
      "toggle_LED_array", &LEDArrayService::LEDCommandStatusCallback,
      LEDArray_Service);
}

void LEDArrayService::SendCommandToHardware(
    LEDSettings* ls, LEDArray* LED_array) {
  ls->cmd = cmd;
  ls->reply = reply;
  for (int8_t i = 0; i < 3; i++) {
    if (ls->cmd == i)
      LED_array->ToggleLEDArray(ls->cmd);
  }
}
