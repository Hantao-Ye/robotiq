#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
ROS node for controling a Robotiq 2F gripper using the Modbus RTU protocol.

The script takes as an argument the IP address of the gripper. It initializes a baseRobotiq2FGripper object and adds a comModbusTcp client to it. It then loops forever, reading the gripper status and updating its command. The gripper status is published on the 'Robotiq2FGripperRobotInput' topic using the 'Robotiq2FGripper_robot_input' msg type. The node subscribes to the 'Robotiq2FGripperRobotOutput' topic for new commands using the 'Robotiq2FGripper_robot_output' msg type. Examples are provided to control the gripper (Robotiq2FGripperSimpleController.py) and interpreting its status (Robotiq2FGripperStatusListener.py).
"""
import rospy
from robotiq_2f_gripper_control.baseRobotiq2FGripper import robotiqbaseRobotiq2FGripper
from robotiq_modbus_rtu.comModbusRtu import communication
import sys
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg


def mainLoop(device, gripper_name):

    # Gripper is a 2F with a TCP connection
    gripper =  robotiqbaseRobotiq2FGripper(communication(True))

    # We connect to the address received as an argument
    print(device)
    if gripper.client.connectToDevice(device) is False:
        rospy.logerr("Failed to connect to the gripper")
        rospy.signal_shutdown("Node useless")
        return

    rospy.init_node("robotiq_2f_gripper_connect")

    # The Gripper status is published on the topic named 'gripper_input'
    pub = rospy.Publisher(gripper_name + "_input",
                          inputMsg.Robotiq2FGripper_robot_input, queue_size=10)

    # The Gripper command is received from the topic named 'gripper_output'
    rospy.Subscriber(gripper_name + "_output",
                     outputMsg.Robotiq2FGripper_robot_output, gripper.refreshCommand)

    # We loop
    while not rospy.is_shutdown():

        # Get and publish the Gripper status
        status = gripper.getStatus()
        pub.publish(status)

        # Wait a little
        # rospy.sleep(0.05)

        # Send the most recent command
        gripper.sendCommand()

        # Wait a little
        # rospy.sleep(0.05)


if __name__ == "__main__":
    try:
        mainLoop(sys.argv[1], sys.argv[2])
    except rospy.ROSInterruptException:
        pass
