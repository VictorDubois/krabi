#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Bool
from time import sleep

def main():
    rclpy.init()
    node = rclpy.create_node('tirette_publisher')
    publisher = node.create_publisher(Bool, 'tirette', 10)
    
    msg = Bool()
    msg.data = True  # Change this to False if you want to publish False

    sleep(2)
    
    while rclpy.ok():
        publisher.publish(msg)
        node.get_logger().info('Published: %r' % msg.data)
        rclpy.spin_once(node)
        sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()