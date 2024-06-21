#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
import control

M = 20
m = 2
l = 0.5
r = 0.025
I = 0.005
g = 9.81

den = I * (M + m) + M * m * l ** 2
A = np.matrix([[0, 1, 0, 0],
               [0, 0, -(m ** 2 * g * l ** 2) / den, 0],
               [0, 0, 0, 1],
               [0, 0, m * g * l * (M + m) / den, 0]
               ])
B = np.matrix([[0], [(I + m * l ** 2) / den], [0], [-m * l / den]])
C = np.matrix([[1, 0, 0, 0], [0, 0, 1, 0]])
D = np.matrix([[0], [0]])

Q = np.diag([1, 1, 1, 1])
R = np.diag([0.1])

K, S, E = control.lqr(A, B, Q, R)


class CartPoleLQR:
    def __init__(self):
        self.pub = None
        self.ctrb = None
        self._rank = None

        self.cartPos = 0
        self.cartvel = 0
        self.polePos = 0
        self.poleVel = 0
        self.ss = None
        self.state = 0
        self.desired_state = np.matrix([[-1], [0], [0], [0]])
        self.init_pub_sub()

    def init_pub_sub(self):
        rospy.Subscriber('/invpend/joint_states', JointState, self._callback)
        self.pub = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=1)

    def _callback(self, data):
        self.polePos = data.position[0]
        self.poleVel = data.velocity[0]
        self.cartPos = data.position[1]
        self.cartvel = data.velocity[1]
        self.state = np.matrix([[self.cartPos], [self.cartvel], [self.polePos], [self.poleVel]])
        self._run_control_loop()

    def _run_control_loop(self):
        self.check_controllability()
        rate = rospy.Rate(50)
        U = -1 * np.matmul(K, (self.state - self.desired_state))
        command = float(U)
        self.pub.publish(command)
        rate.sleep()

    def check_controllability(self):
        self.ss = control.ss(A, B, C, D)
        self.ctrb = control.ctrb(self.ss.A, self.ss.B)
        self._rank = np.linalg.matrix_rank(self.ctrb)
        print("RANK : " + str(self._rank) + ", FULL RANK : 4")


def main():
    rospy.init_node('LQR_Control')
    rospy.loginfo("[LQR_Control] initialised")
    obj = CartPoleLQR()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('[LQR_Control] closed')


if __name__ == '__main__':
    main()
