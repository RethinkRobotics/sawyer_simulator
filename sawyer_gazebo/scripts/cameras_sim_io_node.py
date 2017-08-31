#! /usr/bin/env python

# Copyright (c) 2017, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import json
from intera_core_msgs.msg import (
                                   IODeviceStatus,
                                   IODataStatus,
                                   IOComponentCommand,
                                   IOComponentStatus,
                                   IOStatus,
                                 )

class CameraIOSim(object):
    def __init__(self, name, is_streaming):
        self._name= name
        self._cmd_times = []
        self._state_pub = rospy.Publisher("/io/internal_camera/" + name + "/state",
                                          IODeviceStatus, queue_size=10, latch=True)
        self._command_sub = rospy.Subscriber("/io/internal_camera/" + name + "/command",
                                             IOComponentCommand, self._handle_command)
        self._state_pub.publish(self._construct_state(is_streaming))

    def _construct_state(self, streaming_state, cmd_time=None):
        msg = IODeviceStatus()
        msg.time = rospy.get_rostime()
        msg.device = IOComponentStatus(name=self._name,
            status=IOStatus(tag="ready", id='', detail=json.dumps("")))
        msg.signals.append(IODataStatus(name="camera_streaming",
            format=json.dumps({"role":"output","type":"bool"}),
            data=json.dumps([streaming_state]),
            status=IOStatus(tag="ready", id='', detail=json.dumps(""))
            ))
        if cmd_time:
            self._cmd_times.append(cmd_time)
            self._cmd_times = self._cmd_times[-100:]
            msg.commands = self._cmd_times
        return msg

    def _handle_command(self, msg):
        """
        command topic callback to echo streaming state
        """
        is_streaming = json.loads(msg.args)["signals"]["camera_streaming"]["data"][0]
        self._state_pub.publish(self._construct_state(is_streaming, cmd_time=msg.time))

if __name__ == "__main__":
    rospy.init_node("camera_io_sim")
    rhc = CameraIOSim("right_hand_camera", is_streaming = True)
    hc = CameraIOSim("head_camera", is_streaming = False)
    rospy.spin()
