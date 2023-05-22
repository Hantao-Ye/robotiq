#!/usr/bin/env python3

import sys
import numpy as np

import rospy
import trajectory_msgs

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input as inputMsg


class RobotiqCGripper(object):
    def __init__(self, gripper_name='gripper'):
        self.cur_status = None
        self.cmd_sub = rospy.Subscriber('gripper_close', Bool, self._cmd_cb)
        self.joint_pub = rospy.Publisher(
            '/joint_states', JointState, queue_size=10)
        self.status_sub = rospy.Subscriber(
            gripper_name + '_input', inputMsg, self._status_cb)
        self.cmd_pub = rospy.Publisher(
            gripper_name + '_output', outputMsg, queue_size=10)
        self.r = rospy.Rate(100)
        self.command_sub = rospy.Subscriber(
            gripper_name + '/goal', FollowJointTrajectoryActionGoal, self.callback)
        print('hit')

    def callback(self, msg):
        # print the actual message in its raw format
        print('hot')
        print("what is ", msg.goal.trajectory.points[0].positions, "this")
        pos_msg = list(msg.goal.trajectory.points[0].positions)

        ref_msg = msg.goal.trajectory.points
        position_msg = list(
            msg.goal.trajectory.points[len(ref_msg)-1].positions)
        position_msg = position_msg[0]
        print('final position msg is : ', position_msg)
        position_given = pos_msg[0]
        # rospy.loginfo("%s", msg.trajectory.points.positions)
        # Set up goal
        goal = FollowJointTrajectoryActionGoal()
        pos_val = position_msg
        print("position command is :", str(pos_val))
        pos = int((pos_val/0.8)*230)
        print(pos)
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = int(np.clip(pos, 0, 255))
        print('cmd :', cmd.rPR)
        cmd.rSP = int(np.clip(255./(0.1-0.013) * (0.1-0.013), 0, 255))
        cmd.rFR = int(np.clip(255./(100.-30.) * (100-30.), 0, 255))
        self.cmd_pub.publish(cmd)
        rospy.sleep(0.1)

    def _cmd_cb(self, msg):
        rospy.loginfo('real close btn pressed : {}'.format(msg.data))
        if msg.data == False:
            self.open(block=False)
        elif msg.data == True:
            self.close(block=False)

    def _status_cb(self, msg):
        self.cur_status = msg

    def joint_pos_publisher(self, pos):
        joint_pos = JointState()
        joint_pos.header.stamp = rospy.Time.now()
        joint_pos.name = ['finger_joint']
        joint_pos.position = [pos]

        self.joint_pub.publish(joint_pos)

    def wait_for_connection(self, timeout=-1):
        rospy.sleep(0.1)
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if (timeout >= 0. and rospy.get_time() - start_time > timeout):
                return False
            if self.cur_status is not None:
                return True
            r.sleep()
        return False

    def is_ready(self):
        return self.cur_status.gSTA == 3 and self.cur_status.gACT == 1

    def is_reset(self):
        return self.cur_status.gSTA == 0 or self.cur_status.gACT == 0

    def is_moving(self):
        return self.cur_status.gGTO == 1 and self.cur_status.gOBJ == 0

    def is_stopped(self):
        return self.cur_status.gOBJ != 0

    def object_detected(self):
        return self.cur_status.gOBJ == 1 or self.cur_status.gOBJ == 2

    def get_fault_status(self):
        return self.cur_status.gFLT

    def get_pos(self):
        '''
        change true pos range to virtual joint range
        [0, 0.087] --> [0.8, 0]
        '''
        po = self.cur_status.gPO
        tmp = np.clip(0.087/(13.-230.)*(po-230.), 0.0, 0.087)
        return np.clip(abs(tmp-0.087)/0.087*0.8, 0.0, 0.8)

    def get_req_pos(self):
        pr = self.cur_status.gPR
        return np.clip(0.087/(13.-230.)*(pr-230.), 0, 0.087)

    def is_closed(self):
        return self.cur_status.gPO >= 230

    def is_opened(self):
        return self.cur_status.gPO <= 13

    # in mA
    def get_current(self):
        return self.cur_status.gCU * 0.1

    # if timeout is negative, wait forever
    def wait_until_stopped(self, timeout=-1):
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if (timeout >= 0. and rospy.get_time() - start_time > timeout) or self.is_reset():
                return False
            if self.is_stopped():
                return True
            r.sleep()
        return False

    def wait_until_moving(self, timeout=-1):
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if (timeout >= 0. and rospy.get_time() - start_time > timeout) or self.is_reset():
                return False
            if not self.is_stopped():
                return True
            r.sleep()
        return False

    def reset(self):
        cmd = outputMsg()
        cmd.rACT = 0
        self.cmd_pub.publish(cmd)

    def activate(self, timeout=-1):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = 0
        cmd.rSP = 255
        cmd.rFR = 150
        self.cmd_pub.publish(cmd)
        r = rospy.Rate(30)
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            if timeout >= 0. and rospy.get_time() - start_time > timeout:
                return False
            if self.is_ready():
                return True
            r.sleep()
        return False

    def auto_release(self):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rATR = 1
        self.cmd_pub.publish(cmd)

    ##
    # Goto position with desired force and velocity
    # @param pos Gripper width in meters. [0, 0.087]
    # @param vel Gripper speed in m/s. [0.013, 0.100]
    # @param force Gripper force in N. [30, 100] (not precise)
    def goto(self, pos, vel, force, block=False, timeout=-1):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = int(np.clip((13.-230.)/0.087 * pos + 230., 0, 255))
        cmd.rSP = int(np.clip(255./(0.1-0.013) * (vel-0.013), 0, 255))
        cmd.rFR = int(np.clip(255./(100.-30.) * (force-30.), 0, 255))
        self.cmd_pub.publish(cmd)
        rospy.sleep(0.1)
        if block:
            if not self.wait_until_moving(timeout):
                return False
            return self.wait_until_stopped(timeout)
        return True

    def stop(self, block=False, timeout=-1):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 0
        self.cmd_pub.publish(cmd)
        rospy.sleep(0.1)
        if block:
            return self.wait_until_stopped(timeout)
        return True

    def open(self, vel=0.1, force=100, block=False, timeout=-1):
        if self.is_opened():
            return True
        return self.goto(1.0, vel, force, block=block, timeout=timeout)

    def close(self, vel=0.1, force=100, block=False, timeout=-1):
        if self.is_closed():
            return True
        return self.goto(-1.0, vel, force, block=block, timeout=timeout)


def main(gripper_name):
    rospy.init_node("robotiq_2f_gripper_ctrl")
    gripper = RobotiqCGripper(gripper_name)

    # BUG: Try to fix the wait for connection failure 
    if gripper.wait_for_connection(10) is False:
        rospy.logerr("Failed to connect gripper")
        rospy.signal_shutdown("Node useless")
        return

    is_reset = gripper.is_reset()
    rospy.loginfo('is_reset={}'.format(is_reset))
    if is_reset:
        print("trying to activate")
        gripper.reset()
        activate_status = gripper.activate()
        rospy.loginfo('activate_status={}'.format(activate_status))

    rospy.loginfo('gripper open={}'.format(gripper.open(block=True)))

    while not rospy.is_shutdown():
        cur_pos = gripper.get_pos()
        rospy.logdebug(
            'robotiq_2f_gripper_ctrl_gui| current gripper pos: {}'.format(cur_pos))
        gripper.joint_pos_publisher(cur_pos)
        gripper.r.sleep()


if __name__ == '__main__':
    main(sys.argv[1])
