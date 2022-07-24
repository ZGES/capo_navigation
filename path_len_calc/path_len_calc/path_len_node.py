import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
import math


class PathLen_Node(Node):

    def __init__(self):
        super().__init__('path_len_node')
        self.subscription = self.create_subscription(
            Path,
            'plan',
            self.length_calc,
            10)
        self.subscription  # prevent unused variable warning
    
    def path_len(poses):
        len = 0
        
        prev_x = 0
        prev_y = 0
        for pose in poses:
            x = pose.pose.position.x
            y = pose.pose.position.y

            dist = math.sqrt((x-prev_x)**2 + (y-prev_y)**2)

            len += dist
            prev_x = x
            prev_y = y

        start_time_secs = poses[0].header.stamp.secs
        start_time_nsecs = poses[0].header.stamp.nsecs
        end_time_secs = poses[-1].header.stamp.secs
        end_time_nsecs = poses[-1].header.stamp.nsecs

        d_secs = end_time_secs - start_time_secs
        d_nsecs = end_time_nsecs - start_time_nsecs

        duration = d_secs + d_nsecs * 1e-9

        return len, duration

    def length_calc(self, msg):
        stamp = msg.header.stamp
        poses = msg.poses

        len, duration = self.path_len(poses)

        self.get_logger().info('At time: {}.{} calculated path of length {}m. It took {}s to calculate.'.format(stamp.secs, stamp.nsecs, len, duration))


def main(args=None):
    rclpy.init(args=args)

    path_len_node = PathLen_Node()

    rclpy.spin(path_len_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_len_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()