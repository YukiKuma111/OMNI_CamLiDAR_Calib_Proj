# coding:utf-8
#!/usr/bin/python
     
# Extract images from a bag file.
     
import rosbag
import rospy
import cv2
import argparse
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class ImageCreator():
    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.description='please enter rosbag path, and image save path.'
        parser.add_argument("-b", "--rosbag", help="this is rosbag path", dest="bag_path", type=str, default="../Livox-SDK/ws_livox/data/lidar/0.bag")
        parser.add_argument("-s", "--save", help="this is image save folder path",  dest="save_path", type=str, default="../Livox-SDK/ws_livox/data/photo/")
        parser.add_argument("-t", "--topic", help="this is image topic",  dest="topic", type=str, default="/usb_cam/image_raw")
        parser.add_argument("-r", "--rate", help="this is output image rate",  dest="rate", type=str, default="1")
        args = parser.parse_args()

        self.bridge = CvBridge()
        with rosbag.Bag(args.bag_path, 'r') as bag:  #要读取的bag文件；
            count = 1
            for topic,msg,t in bag.read_messages():
                if topic == args.topic: #图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                        except CvBridgeError as e:
                            print (e)
                        timestr = "%.6f" %  msg.header.stamp.to_sec()   #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        image_name = timestr+ ".png"    #图像命名：时间戳.png

                        if count % int(args.rate) == 0:
                            cv2.imwrite(args.save_path + image_name, cv_image)  #保存；
                            count = 1
                            print("Save ", image_name)
                        else:
                            count += 1
     
if __name__ == '__main__':
    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
    