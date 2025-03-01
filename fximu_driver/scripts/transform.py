import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
import numpy as np
from scipy.spatial.transform import Rotation as R

class MagGlobalTransformNode(Node):
    def __init__(self):
        super().__init__('mag_global_transform_node')

        # Subscriber to /imu/data (for orientation quaternion)
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            250)

        # Subscriber to /imu/mag (for magnetic field vector)
        self.mag_subscription = self.create_subscription(
            MagneticField,
            '/imu/mag',
            self.mag_callback,
            250)

        # Publisher for transformed magnetic field in global frame
        self.global_mag_publisher = self.create_publisher(
            MagneticField,
            '/imu/mag_global',
            250)

        self.additional_rotation = R.from_euler('y', -45, degrees=True)
        self.magAlpha = 0.02
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0

        # Initialize variables to store orientation and magnetic field
        self.orientation_quaternion = None
        self.magnetic_field = None

        self.get_logger().info("Mag Global Transform Node has started.")

    def imu_callback(self, msg):
        # Store the orientation quaternion from /imu/data
        self.orientation_quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        
        # If we have both orientation and magnetic field data, perform the transformation
        if self.magnetic_field is not None:
            self.transform_and_publish()

    def mag_callback(self, msg):
        # Store the magnetic field vector from /imu/mag
        self.magnetic_field = [
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z
        ]
        
        # If we have both orientation and magnetic field data, perform the transformation
        if self.orientation_quaternion is not None:
            self.transform_and_publish()

    def transform_and_publish(self):
        # Ensure we have both orientation and magnetic field data
        if self.orientation_quaternion is None or self.magnetic_field is None:
            return
        
        # Convert the quaternion to a rotation matrix
        rotation = R.from_quat(self.orientation_quaternion)
        
        # Rotate the magnetic field vector into the global frame
        global_mag_vector = rotation.apply(self.magnetic_field)

        # Apply the additional 45-degree rotation around the Y-axis
        #global_mag_vector = self.additional_rotation.apply(global_mag_vector)

        # averaging
        fx = self.magAlpha * global_mag_vector[0] + (1.0 - self.magAlpha) * self.px;
        fy = self.magAlpha * global_mag_vector[1] + (1.0 - self.magAlpha) * self.py;
        fz = self.magAlpha * global_mag_vector[2] + (1.0 - self.magAlpha) * self.pz;

        self.px = fx
        self.py = fy
        self.pz = fz

        # Create a new MagneticField message for the global frame
        global_mag_msg = MagneticField()
        global_mag_msg.header.stamp = self.get_clock().now().to_msg()
        global_mag_msg.header.frame_id = "imu_link"
        global_mag_msg.magnetic_field.x = fx
        global_mag_msg.magnetic_field.y = fy
        global_mag_msg.magnetic_field.z = fz
        
        # Publish the transformed magnetic field
        self.global_mag_publisher.publish(global_mag_msg)

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create the node
    node = MagGlobalTransformNode()
    
    # Spin the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
