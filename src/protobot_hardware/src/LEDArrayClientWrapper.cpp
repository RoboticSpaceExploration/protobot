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

#include "../include/LEDArrayClientWrapper.h"

// how to call LED Array server

int main(int argc, char** argv) {
  // start ROS node
  ros::init(argc, argv, "LED_toggle_client_wrapper_node");

  // instantiate class
  LEDArrayClientWrapper ClientWrapper;

  // toggle LED matrix
  ClientWrapper.ToggleLEDStatus(0);
  sleep(2);
  ClientWrapper.ToggleLEDStatus(1);
  sleep(2);
  ClientWrapper.ToggleLEDStatus(2);
  sleep(2);
  ClientWrapper.ToggleLEDStatus(3);

  // kill node once task is complete
  ros::shutdown();
  return 0;
}

LEDArrayClientWrapper::LEDArrayClientWrapper() {
  client = nh.serviceClient<protobot_hardware::LED_toggle>
      ("LED_toggle_server");
  serverStatus = false;
}

void LEDArrayClientWrapper::ToggleLEDStatus(int cmd) {
  srv.request.LED_toggle = static_cast<int8_t>(cmd);
  CallServer();
}

void LEDArrayClientWrapper::CallServer() {
  serverStatus = client.call(srv);

  if (serverStatus) {
    ROS_INFO("Command sent to LED_toggle_server: [%d]", srv.request.LED_toggle);
    ROS_INFO("Reply: [%d]", srv.response.reply);
  } else {
    ROS_ERROR("[%d] is not a valid command ",
              srv.request.LED_toggle);
    ROS_ERROR("Reply: [%d]", srv.response.reply);
  }
}

