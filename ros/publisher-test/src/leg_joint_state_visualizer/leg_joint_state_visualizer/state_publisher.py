from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

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

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['base_to_hip_axis_3_joint', 'hip_axis_3_to_hip_axis_2_joint', 'hip_axis_2_to_hip_axis_1_joint', 'knee_to_lower_leg_joint']
                joint_state.position = [theta1, theta2, theta3, theta4]

                # update transform
                # # (moving in a circle with radius=2)
                # odom_trans.header.stamp = now.to_msg()
                # odom_trans.transform.translation.x = cos(angle)*2
                # odom_trans.transform.translation.y = sin(angle)*2
                # odom_trans.transform.translation.z = 0.7
                # odom_trans.transform.rotation = \
                #     euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

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

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()