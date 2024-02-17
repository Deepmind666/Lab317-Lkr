import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

subscriberNodeName = 'camera_sensor_subscriber'
topicName = 'video_topic'

def detect_blue_color_blocks(cv_image):
    # 转换到HSV颜色空间
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # 定义蓝色的HSV范围
    lower_blue = np.array([110, 50, 50])
    upper_blue = np.array([130, 255, 255])
    # 创建掩模
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # 绘制轮廓
    cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)
    return cv_image

def callbackFunction(message):
    bridgeObject = CvBridge()
    rospy.loginfo("received a video message/frame")
    convertedFrameBackToCV = bridgeObject.imgmsg_to_cv2(message, desired_encoding='bgr8')
    # 调用颜色块检测函数
    result_image = detect_blue_color_blocks(convertedFrameBackToCV)
    cv2.imshow("Blue Color Block Detection", result_image)
    cv2.waitKey(1)

rospy.init_node(subscriberNodeName, anonymous=True)
rospy.Subscriber(topicName, Image, callbackFunction)

rospy.spin()  # Keep the node running until terminated

cv2.destroyAllWindows()