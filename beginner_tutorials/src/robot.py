#!/usr/bin/env python3

import rospy
import control
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, TwistWithCovariance, PoseWithCovariance
from nav_msgs.msg import Odometry
from beginner_tutorials.msg import Encoders
from tf.transformations import *
from math import sin, cos


class MotorModel:
    def __init__(self):
        k = 1
        T = 1
        W = control.tf(k, [T, 1])
        self.sys = control.LinearIOSystem(W)
        self.x = [[0, 0]]
        self.t = rospy.get_time()
        self.w_target = 0
        self.enc = 0

    def step(self, w_target):
        t = rospy.get_time()
        a, w, self.x = control.input_output_response(self.sys, [self.t, t], [self.w_target, w_target], self.x[0][1], return_x=True)
        dt = t - self.t
        d_enc = int(w[1] * dt / 2 / 3.1415926535 * 4096)

        true_w = d_enc * 2 * 3.1415926535 / dt / 4096

        self.enc += d_enc
        self.t = t
        self.w_target = w_target
        return true_w, dt


class Commander:

    def __init__(self):
        pub1_name = "/enc"
        pub2_name = "/odom"
        sub_name = "/cmd_vel"

        self.pub_enc = rospy.Publisher(pub1_name, Encoders, queue_size=10)
        self.pub_odom = rospy.Publisher(pub2_name, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(sub_name, Twist, self.callback)
        self.pose = PoseWithCovariance()
        self.twist = TwistWithCovariance()
        self.left_motor = MotorModel()
        self.right_motor = MotorModel()
        self.rot = 0


    def callback(self, msg):
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Component: [%f]"%(msg.linear.x))
        rospy.loginfo("Angular Component: [%f]"%(msg.angular.z))
        self.update(msg)


    def update(self, msg):
        L = 0.287
        r = 0.033

        V = msg.linear.x
        Omega = msg.angular.z
        wl = (V - .5*Omega*L)/r
        wr = (V + .5*Omega*L)/r

        true_wl, dt = self.left_motor.step(wl)
        true_wr, dt = self.right_motor.step(wr)

        V = .5*r * (true_wl + true_wr)
        Omega = r/L * (true_wr - true_wl)
        self.rot += Omega*dt

        x = V * cos(self.rot) * dt
        self.pose.pose.position.x += x

        y = V * sin(self.rot) * dt
        self.pose.pose.position.y += y

        q = quaternion_from_euler(0, 0, self.rot)
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]

        self.twist.twist.linear.x = V * cos(self.rot)
        self.twist.twist.linear.y = V * sin(self.rot)
        self.twist.twist.angular.z = self.rot


    def publish_enc(self):
        header = Header(stamp=rospy.Time.now(), 
                        frame_id="enc")
        msg = Encoders(header=header, 
                       left=self.left_motor.enc, 
                       right=self.right_motor.enc)
        
        self.pub_enc.publish(msg)
        rospy.loginfo(msg)


    def publish_odom(self):
        header = Header(stamp=rospy.Time.now(), 
                        frame_id="odom")
        msg = Odometry(header=header, 
                       child_frame_id="odom", 
                       pose=self.pose, 
                       twist=self.twist)
        
        self.pub_odom.publish(msg)
        rospy.loginfo(msg)


if __name__ == '__main__':
    rospy.init_node("Robot")
    rate = rospy.Rate(10) # 10hz
    com = Commander()
    while not rospy.is_shutdown():
        com.publish_enc()
        com.publish_odom()
        rate.sleep()
