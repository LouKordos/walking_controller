from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

import socket

 

localIP     = "127.0.0.1"
localPort   = 4200
bufferSize  = 4096
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Bind to address and ip
UDPServerSocket.bind((localIP, localPort))
print("UDP server up and listening")

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(50)

        theta1 = 0.0
        theta2 = 0.0
        theta3 = 0.0
        theta4 = 0.0

        # message declarations
        # odom_trans = TransformStamped()
        # odom_trans.header.frame_id = 'odom'
        # odom_trans.child_frame_id = 'axis'
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

                message = bytesAddressPair[0]
                
                values = [float(x) for x in message.decode().split("|")]

                theta1 = values[0]
                theta2 = values[1]
                theta3 = values[2]
                theta4 = values[3]

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['base_to_hip_axis_3_joint', 'hip_axis_3_to_hip_axis_2_joint', 'hip_axis_2_to_hip_axis_1_joint', 'knee_to_lower_leg_joint']
                joint_state.position = [theta1, theta2, theta3, theta4]

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                # self.broadcaster.sendTransform(odom_trans)

                # Create new robot state
                # theta1 += 0.001
                # theta2 += 0.001
                # theta3 += 0.001
                # theta4 += 0.001

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()