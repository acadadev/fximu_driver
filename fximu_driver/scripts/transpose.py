import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import numpy as np
import math

class MagTransformNode(Node):
    def __init__(self):
        super().__init__('mag_transform_node')
        
        # Subscriber to /imu/mag
        self.subscription = self.create_subscription(
            MagneticField,
            '/imu/mag',
            self.mag_callback,
            10)
        
        # Publisher to /imu/mag2
        self.publisher = self.create_publisher(
            MagneticField,
            '/imu/mag2',
            10)
        
        self.get_logger().info("Mag Transform Node has started.")

    def mag_callback(self, msg):
        # Extract the magnetic field vector
        x = msg.magnetic_field.x
        y = msg.magnetic_field.y
        z = msg.magnetic_field.z
        
        # Define the rotation angle (-45 degrees around the Y-axis)
        theta = math.radians(-45)  # Negative angle for upward rotation
        
        # Define the rotation matrix for rotation around the Y-axis
        rotation_matrix = np.array([
            [math.cos(theta), 0, math.sin(theta)],
            [0, 1, 0],
            [-math.sin(theta), 0, math.cos(theta)]
        ])
        
        # Apply the rotation to the vector
        original_vector = np.array([x, y, z])
        rotated_vector = np.dot(rotation_matrix, original_vector)
        
        # Extract the rotated components
        rotated_x = rotated_vector[0]
        rotated_y = rotated_vector[1]
        rotated_z = rotated_vector[2]
        
        # Create a new MagneticField message
        rotated_msg = MagneticField()
        rotated_msg.header = msg.header
        rotated_msg.magnetic_field.x = rotated_x
        rotated_msg.magnetic_field.y = rotated_y
        rotated_msg.magnetic_field.z = rotated_z
        
        # Publish the rotated message
        self.publisher.publish(rotated_msg)

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create the node
    node = MagTransformNode()
    
    # Spin the node in a loop
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    
    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
