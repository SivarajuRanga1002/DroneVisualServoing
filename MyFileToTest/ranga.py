#!/usr/bin/python

#Libraries to import
import rospy
import math

	# gathers pose information from the topics
from geometry_msgs.msg import PoseStamped 
	# Communication with Mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
	# Transformations to quaternions from euler
from tf.transformations import quaternion_from_euler


#Class initialization
class FlyDrone:
    
    def __init__(self):

	#Objects
        self.rock = [60.208, -12.502, 18.775]
        self.rover = [12.5, -65.0, -3.5]
        self.probe = [40.732, 3.339, 10.898]

	# Points to visit
        self.pt1 = [0, 0, 15] 
        self.pt2 = [40.852, 3.475, 15]

	#Radius to Maintain from the rock
        self.radius = 4

	#Requests
        self.service_timeout = 30
        self.last_request = 5

	#State and Mode
        self.current_state = False
        self.current_mode = "MANUAL"

	#Attributes
        self.i = 0
        self.angle = 0.0
        self.position_x, self.position_y, self.position_z = 0, 0, 0
        
        self.pose = PoseStamped()

        rospy.init_node("offboard", anonymous=True)

        rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.position_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.drone_pose_cb)

        try:
            rospy.wait_for_service('/mavros/cmd/arming', self.service_timeout)
            rospy.wait_for_service('/mavros/set_mode', self.service_timeout)
            rospy.loginfo('Services are connected and ready')

        except rospy.ROSException as e:
            rospy.logerr('Failed to initialize service')       

        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)


        self.rate = rospy.Rate(20)

        while (not rospy.is_shutdown()) and (self.current_state):

            rospy.spin()
            self.rate.sleep()

    def state_cb(self, data):

        self.current_mode = data.mode
        self.current_state = data.armed

    def drone_pose_cb(self, data):

        self.position_x = data.pose.position.x
        self.position_y = data.pose.position.y
        self.position_z = data.pose.position.z

    def headedto(self, x, y):

        heading = math.atan2((self.position_y - y), (self.position_x - x))
        return heading
        

    def move(self, x, y, z, az):

        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = z

        q = quaternion_from_euler(0.0, 0.0, az)

        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]

        for i in range(100):
            self.position_pub.publish(self.pose)
            self.rate.sleep()      

    def takeoff(self):

        mode = "OFFBOARD"

        while not rospy.is_shutdown():

            if self.current_mode != "OFFBOARD":

                mode_change = self.set_mode_client.call(0, mode)

                if not mode_change.mode_sent:

                    rospy.loginfo("mode change failed")

            if not self.current_state:

                arm_response = self.arming_client.call(True)

                if arm_response:

                    rospy.loginfo("Armed")

                else:

                    rospy.loginfo("failed")

            self.rate = rospy.Rate(10)

            self.move(self.pt1[0], self.pt1[1], self.pt1[2], self.angle)

            if(abs(self.position_z - self.pt1[2]) < 0.5):
                rospy.loginfo("point 1 reached")

            self.move(self.pt2[0], self.pt2[1], self.pt2[2], self.angle)

            if(abs(self.position_z - self.pt2[2]) < 0.5):
                rospy.loginfo("point 2 reached")

            self.move(self.probe[0], self.probe[1], self.probe[2] + 1, self.angle)

            if(abs(self.position_z - self.probe[2]) <= 2):
                rospy.loginfo("Data muling done")
                print("Data muling done")
            
            self.move(self.rock[0] + self.radius + 1.0, self.rock[1] + self.radius + 1.0, self.rock[2] + self.radius + 1.0, self.angle)

            if self.position_z >= self.rock[2] and self.position_x >= self.rock[0]:
                rospy.loginfo("Reached Rock")

                count = 0

                while True:

                    count = count + 1

                    for i in range(0, 360):

                        self.pose.pose.position.x = self.rock[0] + self.radius * math.cos(math.radians(i))
                        self.pose.pose.position.y = self.rock[1] + self.radius * math.sin(math.radians(i))
                        self.pose.pose.position.z = self.rock[2] + 0.25

                        qu = quaternion_from_euler(0.0, 0.0, math.radians(i - 180))

                        self.pose.pose.orientation.x = qu[0]
                        self.pose.pose.orientation.y = qu[1]
                        self.pose.pose.orientation.z = qu[2]
                        self.pose.pose.orientation.w = qu[3]

                        self.position_pub.publish(self.pose)
                        self.rate.sleep()

                    self.rock[2] = self.rock[2] + 0.25

                    if count == 1:
                        self.pose.pose.position.z = self.pose.pose.position.z + 5
                        print("{} --- {}".format(self.position_z, self.pose.pose.position.z))
                        for i in range(40):

                            self.position_pub.publish(self.pose)
                            self.rate.sleep()

                    angle = self.headedto(self.rover[0], self.rover[1])

                    self.move(self.rover[0], self.rover[1], self.rover[2] + 3, angle)

                    if(abs(self.pose.pose.position.z - self.rover[2]+3) < 1):
                        rospy.loginfo("Rover is reached")

                    angle = self.headedto(self.rover[0], self.rover[1])
                    self.move(self.rover[0], self.rover[1], self.rover[2], angle)

                    if (abs(self.pose.pose.position.z - self.rover[2]) == 0  and abs(self.pose.pose.position.x - self.rover[0]) == 0  and abs(self.pose.pose.position.y - self.rover[1] == 0)):

                        self.set_mode_client(0,"AUTO.LAND")

                        if mode_change_response.mode_sent:

                            rospy.loginfo("Ready to perform landing")

                        if self.current_state:

                            arm_response = self.arming_client.call(False)

                            if arm_response:

                                rospy.loginfo("Disarm")

                        break

                break

if __name__ == '__main__':
    try:

        Automate = FlyDrone()
        Automate.takeoff()

    except rospy.ROSInterruptException:

        pass
