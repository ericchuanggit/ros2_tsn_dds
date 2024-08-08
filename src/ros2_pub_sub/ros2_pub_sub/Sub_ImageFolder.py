import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import time
import json

class Listener(Node):

    def __init__(self):
        super().__init__('images_listener')
        self._time = self.get_clock
        #Create subscriber topic 
        self.sub = self.create_subscription(Image,'images_folder',self.listener_callback,10)
        self.subscriptions # prevent unused variable warning
        #Create CvBridge(ros library) convert Opencv image type
        self.subscription_msg = self.create_subscription(String, 'counter', self.listener_callback_msg, 10)
        self.subscription_msg # prevent unused variable warning
        self.bridge = CvBridge()
        self.start_time = 0
        self.end_time = 0

    def listener_callback(self,images):
        # self.get_logger().info("Receiving image")
        # Convert ROS Image message to OpenCV image
        current_image = self.bridge.imgmsg_to_cv2(images)
        # image window
        current_image = cv2.resize(current_image, (1200, 800), interpolation=cv2.INTER_AREA)
        cv2.imshow("image", current_image)
        cv2.waitKey(1)


    def listener_callback_msg(self, msg):
        json_msg = "{"+"{}".format(msg.data)+"}"
        obj = json.loads(json_msg)
        image_count = obj["count"]
        nanosecs = self._time().now()
        _nanosecs=nanosecs.nanoseconds
        time_in_nanosecs = str(_nanosecs)[:10] + '.' + str(_nanosecs)[9:]
        measure_time = float(time_in_nanosecs)-float(obj["time"])
        self.get_logger().info('count: {}, time from send(nanosecs): {}'.format(image_count, float(obj["time"])))
        self.get_logger().info('measure_time(receive-send) {}'.format(measure_time))
        if image_count == "1":
            self.start_time = float(obj["time"])
            print(self.start_time)
        if image_count == "250":
            self.end_time = float(obj["time"])
            total_time = self.end_time - self.start_time
              
            print("total time: {}".format(total_time))
    
def main(args=None):
        rclpy.init(args=args)

        image_subscriber = Listener()
        
        rclpy.spin(image_subscriber)   
        
        image_subscriber.destroy_node()
        
        cv2.destroyAllWindows()
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()  