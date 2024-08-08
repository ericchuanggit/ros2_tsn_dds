import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from datetime import datetime

class TimePublisher(Node):
    def __init__(self):
        super().__init__('time_publisher')
        self.publisher_ = self.create_publisher(String, 'time_topic', 10)
        timer_period = 1  # 每隔1秒發送一次
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        current_time = time.time()
        formatted_current_time = datetime.fromtimestamp(current_time).strftime('%Y %b %d %a,%H:%M:%S')
        
        msg_formatted_current_time = String()
        msg_formatted_current_time.data = str(formatted_current_time)
        
        msg_current_time = String()
        msg_current_time.data = str(current_time)
    
        self.publisher_.publish(msg_current_time)
        self.get_logger().info('Publishing system time: "%s"' % msg_formatted_current_time.data)

def main(args=None):
    rclpy.init(args=args)

    time_publisher = TimePublisher()

    rclpy.spin(time_publisher)

    time_publisher.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
