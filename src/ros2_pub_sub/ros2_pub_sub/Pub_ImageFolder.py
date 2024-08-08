import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import glob
import numpy as np
import os

class Talker(Node):

    def __init__(self,folder_path):

        super().__init__('folder_talker')
        self.send_time = self.get_clock
        self.i = 0 
        
        #Set load image_folder path
        self.folder_path = folder_path

        #Create CvBridge(ros library) convert ros's image type
        self.bridge = CvBridge()

        #Publisher's topic
        self.pub = self.create_publisher(Image, 'images_folder',10)
        self.pub_msg = self.create_publisher(String, 'counter', 10)
        
        #Create timer
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def load_folder_images(self):
        
        input_path_extension = self.folder_path.split('.')[-1]
        
        if input_path_extension in ['jpg', 'jpeg', 'png']:
                return [self.folder_path]
        elif input_path_extension == "txt":
            with open(self.folder_path, "r") as f:
                return f.read().splitlines()
        else:
                return glob.glob(
                os.path.join(self.folder_path, "*.jpg")) + \
                glob.glob(os.path.join(self.folder_path, "*.png")) + \
                glob.glob(os.path.join(self.folder_path, "*.jpeg"))


    def timer_callback(self):
        if self.i == len(self.load_folder_images()):
            rclpy.shutdown()

        images = cv2.imread(self.load_folder_images()[self.i])

        msg = String()
        #Opencv image read 
        
        self.i += 1
        #Show image's number 
        cv2.putText(images, "{}".format(self.i), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 36, 255), 8, cv2.LINE_AA)
        
        #Publish images_folder
        self.pub.publish(self.bridge.cv2_to_imgmsg(images))
        self.get_logger().info('Publishing:{0}'.format(self.i))

        #Publish the message
        nanosecs = self.send_time().now()
        _nanosecs=nanosecs.nanoseconds
        time_in_nanosecs = str(_nanosecs)[:10] + '.' + str(_nanosecs)[9:]
        msg.data = "\"count\":\"{}\",\"time\":\"{}\"".format(self.i, time_in_nanosecs)
        
        self.pub_msg.publish(msg)
        self.get_logger().info('Publishing: {}'.format(msg.data))
        
        
def main(args=None):
    #Initialize the rclpy library
    rclpy.init(args=args)
    
    #User need to load the image folder
    folder_path = input("Please enter the path to images folder : ")

    image_publisher = Talker(folder_path)
    
    #Cyclic sending publisher node
    rclpy.spin(image_publisher)
    
    #Destory the publisher node
    image_publisher.destroy_node()

    #Shutdown the ROS client library for Python
    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
