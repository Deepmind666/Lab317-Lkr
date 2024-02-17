import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

subscriberNodeName = 'camera_sensor_subscriber'
topicName = 'video_topic'

def detect_cracks(cv_image):
    # 将图像转换为灰度图，这是边缘检测所需的
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # 应用高斯模糊，减少图像噪声
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # 使用Canny算法检测边缘
    edges = cv2.Canny(blurred, 50, 150)
    
    # 查找边缘图像中的轮廓
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 在原图上绘制轮廓
    cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)
    
    # 对于每个轮廓，绘制其边界框
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
    
    return cv_image

def callbackFunction(message):
    bridgeObject = CvBridge()
    rospy.loginfo("received a video message/frame")
    # 将接收到的Image消息转换为OpenCV图像格式
    convertedFrameBackToCV = bridgeObject.imgmsg_to_cv2(message, desired_encoding='bgr8')
    # 调用裂缝检测函数
    result_image = detect_cracks(convertedFrameBackToCV)
    # 在窗口中显示结果图像
    cv2.imshow("Crack Detection", result_image)
    # 等待1ms以获取下一个图像帧
    cv2.waitKey(1)

# 初始化ROS节点
rospy.init_node(subscriberNodeName, anonymous=True)
# 创建一个Subscriber对象，它订阅名为video_topic的话题，并在接收到新消息时调用callbackFunction
rospy.Subscriber(topicName, Image, callbackFunction)

# 使ROS节点保持运行，直到它被关闭
rospy.spin()

# 销毁所有OpenCV窗口
cv2.destroyAllWindows()
