import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
import time

class MagTransposeNode(Node):
    def __init__(self):
        super().__init__('mag_transpose_node')
        
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
        
        self.get_logger().info("Mag Transpose Node has started.")

    def mag_callback(self, msg):
        # Extract the magnetic field vector
        x = msg.magnetic_field.x
        y = msg.magnetic_field.y
        z = msg.magnetic_field.z
        
        # Perform transpose (in this case, it's just re-publishing since transpose of a vector is itself)
        # If you meant something else by "transpose," clarify the operation.
        transposed_x = x
        transposed_y = y
        transposed_z = z
        
        # Create a new MagneticField message
        transposed_msg = MagneticField()
        transposed_msg.header = msg.header
        transposed_msg.magnetic_field.x = transposed_x
        transposed_msg.magnetic_field.y = transposed_y
        transposed_msg.magnetic_field.z = transposed_z
        
        # Publish the transposed message
        self.publisher.publish(transposed_msg)

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create the node
    node = MagTransposeNode()
    
    # Spin the node in a loop
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            time.sleep(0.1)  # Add a small delay to avoid busy-waiting
    except KeyboardInterrupt:
        pass
    
    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
