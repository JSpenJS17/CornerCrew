#!/usr/bin/env python3
# encoding: utf-8
# @data:2024/02/28
# @author:pierce
# 颜色检测(color detection)
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

range_rgb = {
    'red':   (  0,  50, 255),
    'blue':  (255,  50,   0),
    'green': ( 50, 255,   0),
    'black': (  0,   0,   0),
    'white': (255, 255, 255),
    'purple':(252,  68, 135), 
    'orange':( 27, 123, 249),
}

class ColorDetectNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.name = name
        self.image = None
        self.running = True
        self.detect_type = {}
        self.target_colors = []
        self.weight_sum = 1.0
        if os.environ['DEPTH_CAMERA_TYPE'] == 'ascamera':
            self.camera_type = 'Stereo'
        else:
            self.camera_type = 'Mono'

        self.image_queue = queue.Queue(maxsize=2)
        self.fps = fps.FPS()  # fps计算器(FPS calculator)
        self.lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
        
        self.bridge = CvBridge()

        line_roi = self.get_parameters_by_prefix('roi_line')
        self.line_roi = {'roi_up': line_roi['roi_up'], 'roi_center': line_roi['roi_center'], 'roi_down': line_roi['roi_down']}
        circle_roi = self.get_parameters_by_prefix('roi_circle')
        self.circle_roi = {'x_min': circle_roi['x_min'].value, 'x_max': circle_roi['x_max'].value, 'y_min': circle_roi['y_min'].value, 'y_max': circle_roi['y_max'].value}
        rect_roi = self.get_parameters_by_prefix('roi_rect')
        self.rect_roi = {'x_min': rect_roi['x_min'].value, 'x_max': rect_roi['x_max'].value, 'y_min': rect_roi['y_min'].value, 'y_max': rect_roi['y_max'].value}
        self.camera = 'ascamera'
        self.display = self.get_parameter('enable_display').value
        self.enable_roi_display = self.get_parameter('enable_roi_display').value

        self.image_sub = self.create_subscription(Image, '/%s/camera_publisher/rgb0/image' % self.camera, self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
        self.info_publisher = self.create_publisher(ColorsInfo, '~/color_info', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 图像处理结果发布(publish the image processing result)

        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)
        self.create_service(SetColorDetectParam, '~/set_param', self.set_param_srv_callback)
        self.create_service(SetLineROI, '~/set_line_roi', self.set_line_roi_srv)
        self.create_service(SetCircleROI, '~/set_circle_roi', self.set_circle_roi_srv)
        self.create_service(SetCircleROI, '~/set_rect_roi', self.set_rect_roi_srv)

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start")
        if self.image_sub is None:
            self.image_sub = self.create_subscription(Image, '/%s/camera_publisher/rgb0/image' % self.camera, self.image_callback, 1)
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop")
        if self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
        response.success = True
        response.message = "stop"
        return response

    def set_circle_roi_srv(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_circle_roi")
        self.circle_roi['x_min'] = request.data.x_min
        self.circle_roi['x_max'] = request.data.x_max
        self.circle_roi['y_min'] = request.data.y_min
        self.circle_roi['y_max'] = request.data.y_max
        response.success = True
        response.message = "set_circle_roi"
        return response

    def set_rect_roi_srv(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_rect_roi")
        self.rect_roi['x_min'] = request.data.x_min
        self.rect_roi['x_max'] = request.data.x_max
        self.rect_roi['y_min'] = request.data.y_min
        self.rect_roi['y_max'] = request.data.y_max
        response.success = True
        response.message = "set_rect_roi"
        return response

    def set_line_roi_srv(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_line_roi")
        roi_up = request.data.roi_up
        roi_center = request.data.roi_center
        roi_down = request.data.roi_down
        self.line_roi['roi_up'] = [roi_up.y_min, roi_up.y_max, roi_up.x_min, roi_up.x_max, roi_up.scale]
        self.line_roi['roi_center'] = [roi_center.y_min, roi_center.y_max, roi_center.x_min, roi_center.x_max, roi_center.scale]
        self.line_roi['roi_down'] = [roi_down.y_min, roi_down.y_max, roi_down.x_min, roi_down.x_max, roi_down.scale]
        response.success = True
        response.message = "set_line_roi"
        return response

    def set_param_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_param")
        if len(request.data) == 1:
            self.target_colors = [request.data[0].color_name, ]
            self.detect_type[request.data[0].color_name] = request.data[0].detect_type
        else:
            self.target_colors = []
            for i in request.data:
                self.get_logger().info('\033[1;32m%s\033[0m' % str([i.color_name, i.detect_type]))
                self.target_colors.append(i.color_name)
                self.detect_type[i.color_name] = i.detect_type
        response.success = True
        response.message = "set_param"
        return response

    def main(self):
        while self.running:
            t1 = time.time()
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            result_image = image.copy()
            h, w = image.shape[:2]
            img_lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)  # bgr转lab(convert from BGR to LAB)
            img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪(gaussian blur for noise reduction)

            centroid_sum = 0

            color_contours = {}
            for i in self.target_colors:
                if self.enable_roi_display:
                    cv2.rectangle(result_image, (self.circle_roi['x_min'], self.circle_roi['y_min']), (self.circle_roi['x_max'], self.circle_roi['y_max']), (0, 255, 255), 1) 
                blob = img_blur[self.circle_roi['y_min']:self.circle_roi['y_max'], self.circle_roi['x_min']:self.circle_roi['x_max']]
                mask = cv2.inRange(blob, tuple(self.lab_data['lab'][self.camera_type][i]['min']), tuple(self.lab_data['lab'][self.camera_type][i]['max']))  # 二值化(binarization)
                eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(erode)
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find contours)
                max_contour_area = common.get_area_max_contour(contours, 0)[0]  # 获取最大面积对应轮廓(find the contour with the largest area)
                
                if max_contour_area is not None: # did we find the color?
                    color_contours[i] = max_contour_area

            colors_info = ColorsInfo()
            colors_info.data = []
            for color, contour_size in color_contours.items():
                ((x, y), radius) = cv2.minEnclosingCircle(contour_size)  # 获取最小外接圆(obtain the minimum circumscribed circle)
                x = int(x) + self.circle_roi['x_min']
                y = int(y) + self.circle_roi['y_min']
                radius = int(radius)
                cv2.circle(result_image, (x, y), radius, range_rgb[color], 2)  # 画圆(draw circle)

                color_info = ColorInfo()
                color_info.width = w
                color_info.height = h
                color_info.color = color
                color_info.x = x
                color_info.y = y
                color_info.radius = radius
                colors_info.data.append(color_info)

            self.info_publisher.publish(colors_info)
            self.fps.update()
            if self.display:
                cv2.imshow("result", result_image)
                cv2.waitKey(1)
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
            t2 = time.time()
            t = t2 - t1
            if t < 0.03:
                time.sleep(0.03 - t)

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put(bgr_image)

def main():
    node = ColorDetectNode('color_detect')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
