import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

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

def main(args=None):
    rclpy.init(args=args)
    node = MagTransposeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
