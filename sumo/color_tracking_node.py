#!/usr/bin/env python3
# encoding: utf-8
# @data:2024/02/28
# @author:pierce

# import everything
import os
import cv2
import time
import math
import queue
import rclpy
import threading
import numpy as np
import sdk.fps as fps
import sdk.common as common
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from interfaces.msg import ColorInfo, ColorsInfo
from interfaces.srv import SetColorDetectParam, SetCircleROI, SetLineROI

class ColorDetectNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # init motor publisher (dancing! :D)
        self.motor_pub = self.create_publisher(MotorsState, 'ros_robot_controller/set_motor', 1)
        # init mecanum chassis controller object
        self.mecanum = MecanumChassis()    

        # declare class variables
        self.name = name
        self.image = None
        self.running = True
        self.detect_type = {}
        self.target_colors = []
        self.weight_sum = 1.0

        # set camera type depending on what camera is available on ROS2
        if os.environ['DEPTH_CAMERA_TYPE'] == 'ascamera':
            self.camera_type = 'Stereo'
        else:
            self.camera_type = 'Mono'

        # init image queue to allow buffer
        self.image_queue = queue.Queue(maxsize=2)

        # FPS calculator
        self.fps = fps.FPS()  
        self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
        
        # init CvBridge for computer vision processing
        self.bridge = CvBridge()


        # enable display parameters
        rect_roi = self.get_parameters_by_prefix('roi_rect')
        self.rect_roi = {'x_min': rect_roi['x_min'].value, 'x_max': rect_roi['x_max'].value, 'y_min': rect_roi['y_min'].value, 'y_max': rect_roi['y_max'].value}
        self.camera = 'ascamera'
        self.display = self.get_parameter('enable_display').value
        self.enable_roi_display = self.get_parameter('enable_roi_display').value

        # subscription to camera output
        self.image_sub = self.create_subscription(Image, '/%s/camera_publisher/rgb0/image' % self.camera, self.image_callback, 1)
        
        # publisher of color information
        self.info_publisher = self.create_publisher(ColorsInfo, 'color_info', 1)

        # publisher of image result
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)

        # create services with callbacks
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)

        # run self.main on its own thread headlessly
        threading.Thread(target=self.main).start()
        # self.create_service(Trigger, '~/init_finish', self.get_node_state)



    def get_node_state(self, request, response):
        response.success = True
        return response


    # start service callback
    def start_srv_callback(self, request, response):
        if self.image_sub is None:
            self.image_sub = self.create_subscription(Image, '/%s/camera_publisher/rgb0/image' % self.camera, self.image_callback, 1)
        response.success = True
        response.message = "start"
        return response

    # stop service callback
    def stop_srv_callback(self, request, response):
        if self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
        response.success = True
        response.message = "stop"
        return response

    # runs on its own thread headlessly
    def main(self):
        while self.running:
            t1 = time.time()
            # try to get an image from the queue
            try:
                image = self.image_queue.get(block=True, timeout=1)

            except queue.Empty:
                # if nothing in image queue and not running,
                if not self.running:
                    # die
                    break
                else:
                    # otherwise, skip a cycle
                    continue

            # grab a copy of the image so we can modify it
            result_image = image.copy()
            # grab height, width of image
            h, w = image.shape[:2]
            # convert from BGR to LAB
            img_lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
            # use gaussian blur for noise reduction
            img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)

            # define variables for upcoming detection loop
            centroid_sum = 0
            max_area_rect = 0
            color_area_max_rect = ''
            areaMaxContour_rect = None

            # loop through target colors
            for i in self.target_colors:
                
                    # detecting rectangles
                    if self.enable_roi_display:
                        cv2.rectangle(result_image, (self.rect_roi['x_min'], self.rect_roi['y_min']), (self.rect_roi['x_max'], self.rect_roi['y_max']), (0, 255, 255), 1) 
                    blob = img_blur[self.rect_roi['y_min']:self.rect_roi['y_max'], self.rect_roi['x_min']:self.rect_roi['x_max']]
                    
                    # shared code between rects and circles
                    mask = cv2.inRange(blob, tuple(self.lab_data['lab'][self.camera_type][i]['min']), tuple(self.lab_data['lab'][self.camera_type][i]['max']))  # 二值化(binarization)
                    eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(erode)
                    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
                    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find contours)
                    max_contour_area = common.get_area_max_contour(contours, 0)[0]  # 获取最大面积对应轮廓(find the contour with the largest area)
                    
                    if max_contour_area is not None:
                        # found countour of color
                        area = math.fabs(cv2.contourArea(max_contour_area))
                        if area > max_area_rect:  # 找最大面积(find the maximum area)
                            max_area_rect = area
                            color_area_max_rect = i
                            areaMaxContour_rect = max_contour_area

            # out of color detection loop
            colors_info = ColorsInfo()
            color_info_list = []

            # set color_info variables to tell where to draw the circle (?)
            rect = cv2.minAreaRect(areaMaxContour_rect)  # 最小外接矩形(the minimum bounding rectangle)
            box = np.intp(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点(four corner points of the minimum bounding rectangle)
            for i in range(4):
                box[i, 0] = box[i, 0] + self.rect_roi['x_min']
                box[i, 1] = box[i, 1] + self.rect_roi['y_min']
            cv2.drawContours(result_image, [box], -1, common.range_rgb[color_area_max_rect], 2)  # 画出四个点组成的矩形(draw the rectangle composed of the four points)

            # 获取矩形对角点(obtain the diagonal points of the rectangle)
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]
            # 中心点(center point)
            x, y = int((pt1_x + pt3_x) / 2), int((pt1_y + pt3_y) / 2)

            color_info = ColorInfo()
            color_info.width = w
            color_info.height = h
            color_info.color = color_area_max_rect
            color_info.x = x
            color_info.y = y
            color_info.angle = int(rect[2])
            color_info_list.append(color_info)

            # update colors_info with our contour data
            colors_info.data = color_info_list

            # publish that countour data to info_publisher
            self.info_publisher.publish(colors_info)

            # update based on how long this loop took to maintain steady fps
            self.fps.update()

            # if we're displaying stuff
            if self.display:
                # show the resulting image with the colors highlighted
                cv2.imshow("result", result_image)
                cv2.waitKey(1)

            # publish the result image to the result_publisher
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))

            # maintain steady FPS (1 frame per .03 s)
            t2 = time.time()
            t = t2 - t1
            if t < 0.03:
                time.sleep(0.03 - t)

    # to put an image in queue
    def image_callback(self, ros_image):
        # convert camera image to CV image
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        # also grab the BGR image (converted to LAB later)
        bgr_image = np.array(cv_image, dtype=np.uint8)
        # if the queue is full, discard the oldest image
        if self.image_queue.full():
            self.image_queue.get()

        # finally, put the image into the queue
        self.image_queue.put(bgr_image)


    def set_velocity(self, linear_x, linear_y, angular_z):
        speeds = self.mecanum.set_velocity(linear_x, linear_y, angular_z)
        self.motor_pub.publish(speeds)


# Grabbed straight from their code
class MecanumChassis:
    # wheelbase = 0.1368   # 前后轴距(distance between front and real axles)
    # track_width = 0.1446 # 左右轴距(distance between left and right axles)
    # wheel_diameter = 0.065  # 轮子直径(wheel diameter)
    def __init__(self, wheelbase=0.1368, track_width=0.1410, wheel_diameter=0.065):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_covert(self, speed):
        """
        covert speed m/s to rps/s
        :param speed:
        :return:
        """
        # distance / circumference = rotations per second
        return speed / (math.pi * self.wheel_diameter)

    def set_velocity(self, linear_x, linear_y, angular_z):
        """
        Use polar coordinates to control moving
                    x
        v1 motor1|  ↑  |motor3 v3
          +  y - |     |
        v2 motor2|     |motor4 v4
        :param speed: m/s
        :param direction: Moving direction 0~2pi, 1/2pi<--- ↑ ---> 3/2pi
        :param angular_rate:  The speed at which the chassis rotates rad/sec
        :param fake:
        :return:
        """
        # vx = speed * math.sin(direction)
        # vy = speed * math.cos(direction)
        # vp = angular_rate * (self.wheelbase + self.track_width) / 2
        # v1 = vx - vy - vp
        # v2 = vx + vy - vp
        # v3 = vx + vy + vp
        # v4 = vx - vy + vp
        # v_s = [self.speed_covert(v) for v in [v1, v2, -v3, -v4]]
        motor1 = (linear_x - linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor2 = (linear_x + linear_y - angular_z * (self.wheelbase + self.track_width) / 2)
        motor3 = (linear_x + linear_y + angular_z * (self.wheelbase + self.track_width) / 2)
        motor4 = (linear_x - linear_y + angular_z * (self.wheelbase + self.track_width) / 2)

        v_s = [self.speed_covert(v) for v in [-motor1, -motor2, motor3, motor4]]
        data = []
        for i in range(len(v_s)):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = float(v_s[i])
            data.append(msg)
        
        msg = MotorsState()
        msg.data = data
        return msg



def main():
    # make the node
    node = ColorDetectNode('color_detect')

    
    # spin on it
    rclpy.spin(node)
    node.destroy_node()

    # uninit
    rclpy.shutdown()

if __name__ == "__main__":
    main()

