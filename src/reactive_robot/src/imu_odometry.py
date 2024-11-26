#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Quaternion
import tf

class OdometryNode:
    def __init__(self):
        # ROS Node başlat
        rospy.init_node('imu_odometry', anonymous=True)

        # Publisher ve Subscriber tanımla
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # tf broadcaster tanımla
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Robotun durum bilgileri
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Robotun yönelimi (radyan)

        # Robotun hareket bilgileri
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Zaman bilgisi
        self.last_time = rospy.Time.now()

        rospy.loginfo("IMU Odometry Node started.")
        rospy.spin()

    def cmd_vel_callback(self, msg):
        """ /cmd_vel topic'inden gelen hız bilgilerini güncelle """
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def imu_callback(self, imu_data):
        """ /imu topic'inden gelen verileri kullanarak odometri hesapla """
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # IMU'dan alınan açısal hız (z ekseni)
        angular_velocity_z = imu_data.angular_velocity.z

        # Robotun pozisyonunu güncelle
        delta_theta = angular_velocity_z * dt
        self.theta += delta_theta

        delta_x = self.linear_velocity * math.cos(self.theta) * dt
        delta_y = self.linear_velocity * math.sin(self.theta) * dt
        self.x += delta_x
        self.y += delta_y

        # Odometri mesajı oluştur
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"

        # Pozisyon bilgisi
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(*self.euler_to_quaternion(0, 0, self.theta))

        # Hız bilgisi
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity_z

        # Odometri mesajını yayınla
        self.odom_pub.publish(odom_msg)

        # tf dönüşümünü yayınla
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),
            self.euler_to_quaternion(0, 0, self.theta),
            current_time,
            "base_link",
            "odom"
        )

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """ Euler açılarını kuaterniyona dönüştür """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

if __name__ == '__main__':
    try:
        OdometryNode()
    except rospy.ROSInterruptException:
        pass


