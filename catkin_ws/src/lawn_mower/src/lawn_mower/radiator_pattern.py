#!/usr/bin/env python

import rospy
import math
import sys, signal
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import (TeleportAbsolute, TeleportAbsoluteRequest)


class ReachGoal2:

    def __init__(self, goal_points):
        self.xp = 0
        self.yp = 0
        self.yaw = 0
        self.line_vel = 0
        self.ang_vel = 0
        self.pub = None
        self.r = rospy.Rate(100)
        self.command = Twist()

        self.parameters = [0.9, 0.0001, 1.5, 3, 0.0001, 0.8]

        self.linear_acc_limit = (10, -10)
        self.angular_acc_limit = (1, -1)

        self.chase_distance = 10
        self.distance_int = 0
        self.yaw_int = 0
        self.yaw_diff = 0
        self.goal_points = goal_points
        self.goal_x = 0
        self.goal_y = 0
        self.record_x = []
        self.record_y = []
        self.record_linear_speed = [0]
        self.record_angular_speed = [0]
        self.n_iter = 100
        self.init_sub_pub()
        self._reach_goal()

    def init_sub_pub(self):
        rospy.Subscriber('/turtle1/pose', Pose, self._cb_input)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

    def _cb_input(self, req):
        self.xp = req.x
        self.yp = req.y
        self.record_x.append(self.xp)
        self.record_y.append(self.yp)
        self.yaw = req.theta
        self.line_vel = req.linear_velocity
        self.record_linear_speed.append(self.line_vel)
        self.ang_vel = req.angular_velocity
        self.record_angular_speed.append(self.ang_vel)

    def _reach_goal(self):
        while True:
            for i in range(0, len(self.goal_points)):
                self.goal_x = self.goal_points[i][0]
                self.goal_y = self.goal_points[i][1]
                print('Current Goal ', self.goal_x, self.goal_y)
                self.rotate()
                print('Rotate Complete')
                self.move_to()
                print('Move complete')

    def move_to(self):
        self.chase_distance = math.sqrt((self.xp - self.goal_x) ** 2 + (self.yp - self.goal_y) ** 2)
        while self.chase_distance > 0.09:
            self.command.linear.x = self._set_linear_speed()
            # self.command.linear.x = 5
            self.command.angular.z = 0.0
            self.pub.publish(self.command)
            self.r.sleep()

    def rotate(self):
        target_rad = math.atan2(self.goal_y - self.yp, self.goal_x - self.xp)
        self.yaw_diff = target_rad - self.yaw
        print('YAW diff : ', self.yaw_diff)
        while self.yaw_diff > 0.01 or self.yaw_diff < -0.01:
            self.command.angular.z = self._set_angular_speed()
            self.command.linear.x = 0.0
            self.pub.publish(self.command)
            self.r.sleep()

    def _set_linear_speed(self):
        self.chase_distance = math.sqrt((self.xp - self.goal_x) ** 2 + (self.yp - self.goal_y) ** 2)
        self.distance_int = self.distance_int + self.chase_distance
        command_linear_speed = (self.parameters[0] * self.chase_distance) + ((-self.parameters[2]) * self.line_vel) + \
                               (self.parameters[1] * self.distance_int)

        linear_acc = (command_linear_speed - self.record_linear_speed[-1]) / 0.01

        if linear_acc > self.linear_acc_limit[0]:
            linear_acc = self.linear_acc_limit[0]
            command_linear_speed = (linear_acc * 0.01) + self.record_linear_speed[-1]

        if linear_acc < self.linear_acc_limit[1]:
            linear_acc = self.linear_acc_limit[1]
            command_linear_speed = (linear_acc * 0.01) + self.record_linear_speed[-1]

        if command_linear_speed >= 1.0:
            command_linear_speed = 1.0

        print('Speed ', command_linear_speed)

        return command_linear_speed

    def _set_angular_speed(self):
        target_rad = math.atan2(self.goal_y - self.yp, self.goal_x - self.xp)
        self.yaw_diff = target_rad - self.yaw
        self.yaw_int = self.yaw_int + self.yaw_diff

        if math.degrees(self.yaw_diff) >= 180:
            self.yaw_diff = self.yaw_diff - 6.2832
        if math.degrees(self.yaw_diff) <= -180:
            self.yaw_diff = self.yaw_diff + 6.2832

        command_angular_speed = (self.parameters[3] * self.yaw_diff) + ((-self.parameters[5]) * self.ang_vel) + \
                                (self.parameters[4] * self.yaw_int)
        angular_acc = (command_angular_speed - self.record_angular_speed[-1]) / 0.01

        if angular_acc > self.angular_acc_limit[0]:
            angular_acc = self.angular_acc_limit[0]
            command_angular_speed = (angular_acc * 0.01) + self.record_angular_speed[-1]

        if angular_acc < self.angular_acc_limit[1]:
            angular_acc = self.angular_acc_limit[1]
            command_angular_speed = (angular_acc * 0.01) + self.record_angular_speed[-1]
        return command_angular_speed


def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('GoRadiator')
    rospy.loginfo("[GoRadiator] initialized")
    call_reset = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
    setr = TeleportAbsoluteRequest()
    start_x = 3.0
    start_y = 3.0
    setr.x = start_x
    setr.y = start_y
    setr.theta = 0
    response = call_reset(setr)
    a_dist = 6.0
    b_dist = 3.0
    params = [10, 0.0001, 0.8, 2, 0.0001, 0.8]
    goal_points = [[start_x + a_dist, start_y], [start_x + a_dist, start_y + b_dist], [start_x, start_y + b_dist],
                   [start_x, start_y + (2 * b_dist)], [start_x + a_dist, start_y + (2 * b_dist)], [start_x, start_y]]
    obj = ReachGoal2(goal_points)

    try:
        print('Spinning')
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('[GoRadiator] closed')


if __name__ == '__main__':
    main()
