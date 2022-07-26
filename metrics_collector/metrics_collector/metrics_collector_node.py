import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from builtin_interfaces.msg import Duration
import math


class Metrics_Collector_Node(Node):

    def __init__(self):
        super().__init__('metrics_collector_node')

        self.path_sub = self.create_subscription(
            Path,
            'plan',
            self.path_collect,
            10)
        self.path_sub  # prevent unused variable warning

        self.duration_sub = self.create_subscription(
            Duration,
            'duration',
            self.duration_collect,
            10)
        self.duration_sub  # prevent unused variable warning
    
    def path_len(self, poses):
        len = 0
        
        prev_x = poses[0].pose.position.x
        prev_y = poses[0].pose.position.y
        for pose in poses[1::]:
            x = pose.pose.position.x
            y = pose.pose.position.y

            dist = math.sqrt((x-prev_x)**2 + (y-prev_y)**2)

            len += dist
            prev_x = x
            prev_y = y

        return len


    def path_collect(self, path):
        stamp = path.header.stamp
        poses = path.poses

        len = self.path_len(poses)

        self.get_logger().info('[PATH] Calculated path of length {}m at timestamp {}.{}s'.format(len, stamp.sec, stamp.nanosec))


    def duration_collect(self, duration):
        self.get_logger().info('[DURATION] It took {}.{}s to plan the path'.format(duration.sec, duration.nanosec))


    def save_position(self, odom):
        self.currX = odom.pose.pose.position.x
        self.currY = odom.pose.pose.position.y

def main(args=None):
    rclpy.init(args=args)

    metrics_collector_node = Metrics_Collector_Node()

    rclpy.spin(metrics_collector_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    metrics_collector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()