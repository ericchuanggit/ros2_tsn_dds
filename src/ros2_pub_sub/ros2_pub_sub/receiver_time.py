import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from datetime import datetime

class TimeListener(Node):
    def __init__(self):
        super().__init__('time_listener')
        self.subscription = self.create_subscription(String, 'time_topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        received_time = float(msg.data)
        formatted_received_time= datetime.fromtimestamp(received_time).strftime('%Y %b %d %a,%H:%M:%S')

        current_time = time.time()
        formatted_current_time = datetime.fromtimestamp(current_time).strftime('%Y %b %d %a,%H:%M:%S')

        time_difference = current_time - received_time
        
        self.get_logger().info('Received host_system time: "%s"' % formatted_received_time)
        self.get_logger().info('KR260 system time:         "%s"' % formatted_current_time)
        self.get_logger().info('Time difference: "%s"' % time_difference)

def main(args=None):
    rclpy.init(args=args)

    time_listener = TimeListener()

    rclpy.spin(time_listener)

    time_listener.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
