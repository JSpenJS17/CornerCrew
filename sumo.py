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
        line_roi = self.get_parameters_by_prefix('roi_line')
        self.line_roi = {'roi_up': line_roi['roi_up'], 'roi_center': line_roi['roi_center'], 'roi_down': line_roi['roi_down']}
        circle_roi = self.get_parameters_by_prefix('roi_circle')
        self.circle_roi = {'x_min': circle_roi['x_min'].value, 'x_max': circle_roi['x_max'].value, 'y_min': circle_roi['y_min'].value, 'y_max': circle_roi['y_max'].value}
        rect_roi = self.get_parameters_by_prefix('roi_rect')
        self.rect_roi = {'x_min': rect_roi['x_min'].value, 'x_max': rect_roi['x_max'].value, 'y_min': rect_roi['y_min'].value, 'y_max': rect_roi['y_max'].value}
        self.camera = 'ascamera'
        self.display = self.get_parameter('enable_display').value
        self.enable_roi_display = self.get_parameter('enable_roi_display').value

        # subscription to camera output
        self.image_sub = self.create_subscription(Image, '/%s/camera_publisher/rgb0/image' % self.camera, self.image_callback, 1)
        
        # publisher of color information
        self.info_publisher = self.create_publisher(ColorsInfo, '~/color_info', 1)

        # publisher of image result
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)

        # create services with callbacks
        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)
        self.create_service(SetColorDetectParam, '~/set_param', self.set_param_srv_callback)
        self.create_service(SetLineROI, '~/set_line_roi', self.set_line_roi_srv)
        self.create_service(SetCircleROI, '~/set_circle_roi', self.set_circle_roi_srv)
        self.create_service(SetCircleROI, '~/set_rect_roi', self.set_rect_roi_srv)

        # run self.main on its own thread headlessly
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)

        # log that we started
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response


    # start service callback
    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start")
        if self.image_sub is None:
            self.image_sub = self.create_subscription(Image, '/%s/camera_publisher/rgb0/image' % self.camera, self.image_callback, 1)
        response.success = True
        response.message = "start"
        return response

    # stop service callback
    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop")
        if self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None
        response.success = True
        response.message = "stop"
        return response

    # set the region of interest as a circle when asked (callback)
    def set_circle_roi_srv(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_circle_roi")
        self.circle_roi['x_min'] = request.data.x_min
        self.circle_roi['x_max'] = request.data.x_max
        self.circle_roi['y_min'] = request.data.y_min
        self.circle_roi['y_max'] = request.data.y_max
        response.success = True
        response.message = "set_circle_roi"
        return response

    # set the region of interest as a rectangle when asked (callback)
    def set_rect_roi_srv(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_rect_roi")
        self.rect_roi['x_min'] = request.data.x_min
        self.rect_roi['x_max'] = request.data.x_max
        self.rect_roi['y_min'] = request.data.y_min
        self.rect_roi['y_max'] = request.data.y_max
        response.success = True
        response.message = "set_rect_roi"
        return response

    # set the region of interest to a line 
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

    # set target colors to color names (orange ?)
    def set_param_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_param")

        # only difference between if and else is for debug
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
            line_color = None

            # rectangle color tracking vars
            max_area_rect = 0
            color_area_max_rect = ''
            areaMaxContour_rect = None

            # circle color tracking vars
            max_area_circle = 0
            color_area_max_circle = ''
            areaMaxContour_circle = None

            # loop through target colors
            for i in self.target_colors:
                # if we're doing color tracking on a line
                if self.detect_type[i] == 'line' and i != '':
                    line_color = i
                    for roi in self.line_roi:
                        ## detect colors on line
                        roi_value = self.line_roi[roi]
                        blob = img_blur[roi_value[0]:roi_value[1], roi_value[2]:roi_value[3]]  # 截取roi(crop ROI)
                        mask = cv2.inRange(blob, tuple(self.lab_data['lab'][self.camera_type][i]['min']), tuple(self.lab_data['lab'][self.camera_type][i]['max']))  # 二值化(binarization)
                        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(erode)
                        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
                        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find contours)
                        max_contour_area = common.get_area_max_contour(contours, 0)[0]  # 获取最大面积对应轮廓(find the contour with the largest area)
                        if max_contour_area is not None:
                            rect = cv2.minAreaRect(max_contour_area)  # 最小外接矩形(find the minimum bounding rectangle)
                            box = np.intp(cv2.boxPoints(rect))  # 四个角(four corners)
                            for j in range(4):
                                box[j, 1] = box[j, 1] + roi_value[0]
                            cv2.drawContours(result_image, [box], -1, common.range_rgb[i], 2)  # 画出四个点组成的矩形(draw the rectangle composed of the four points)

                            # 获取矩形对角点(obtain the diagonal points of the rectangle)
                            pt1_x, pt1_y = box[0, 0], box[0, 1]
                            pt3_x, pt3_y = box[2, 0], box[2, 1]
                            # 线的中心点(the center point of the line)
                            line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                            cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, common.range_rgb[i], -1)  # 画出中心点(draw the center point)
                            centroid_sum += line_center_x * roi_value[-1]
                elif i != '':
                    # detecting rectangles
                    if self.detect_type[i] == 'rect':
                        if self.enable_roi_display:
                            cv2.rectangle(result_image, (self.rect_roi['x_min'], self.rect_roi['y_min']), (self.rect_roi['x_max'], self.rect_roi['y_max']), (0, 255, 255), 1) 
                        blob = img_blur[self.rect_roi['y_min']:self.rect_roi['y_max'], self.rect_roi['x_min']:self.rect_roi['x_max']]
                    # detecting circles
                    else:
                        if self.enable_roi_display:
                            cv2.rectangle(result_image, (self.circle_roi['x_min'], self.circle_roi['y_min']), (self.circle_roi['x_max'], self.circle_roi['y_max']), (0, 255, 255), 1) 
                        blob = img_blur[self.circle_roi['y_min']:self.circle_roi['y_max'], self.circle_roi['x_min']:self.circle_roi['x_max']]
                    
                    # shared code between rects and circles
                    mask = cv2.inRange(blob, tuple(self.lab_data['lab'][self.camera_type][i]['min']), tuple(self.lab_data['lab'][self.camera_type][i]['max']))  # 二值化(binarization)
                    eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(erode)
                    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
                    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find contours)
                    max_contour_area = common.get_area_max_contour(contours, 0)[0]  # 获取最大面积对应轮廓(find the contour with the largest area)
                    
                    if max_contour_area is not None:
                        # found countour of color
                        area = math.fabs(cv2.contourArea(max_contour_area))
                        # if it was a rectangle,
                        if self.detect_type[i] == 'rect':
                            if area > max_area_rect:  # 找最大面积(find the maximum area)
                                max_area_rect = area
                                color_area_max_rect = i
                                areaMaxContour_rect = max_contour_area
                        # if it was a circle,
                        else:
                            if area > max_area_circle:  # 找最大面积(find the maximum area)
                                max_area_circle = area
                                color_area_max_circle = i
                                areaMaxContour_circle = max_contour_area
            
            # out of color detection loop
            colors_info = ColorsInfo()
            color_info_list = []
            center_pos = centroid_sum / self.weight_sum  # 按比重计算中心点(calculate the center point based on weighted average)
            
            # if we were a line and we found the color in the image
            if line_color is not None and int(center_pos) != 0:
                # set color_info variables to tell where to draw the circle (?)
                color_info = ColorInfo()
                color_info.width = w
                color_info.height = h
                color_info.color = line_color
                color_info.x = int(center_pos)
                color_info.y = h - 50
                color_info_list.append(color_info)
            
            # if we were a rectangle and found the color
            if areaMaxContour_rect is not None:
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
            
            # if we were a circle and found the color
            if areaMaxContour_circle is not None:
                # set color_info variables to tell where to draw the circle (?)
                ((x, y), radius) = cv2.minEnclosingCircle(areaMaxContour_circle)  # 获取最小外接圆(obtain the minimum circumscribed circle)
                x = int(x) + self.circle_roi['x_min']
                y = int(y) + self.circle_roi['y_min']
                radius = int(radius)
                cv2.circle(result_image, (x, y), radius, common.range_rgb[color_area_max_circle], 2)  # 画圆(draw circle)

                color_info = ColorInfo()
                color_info.width = w
                color_info.height = h
                color_info.color = color_area_max_circle
                color_info.x = x
                color_info.y = y
                color_info.radius = radius
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


def main():
    # make the node
    node = ColorDetectNode('color_detect')
    # spin on it
    rclpy.spin(node)
    # die when we're done (Ctrl+C)
    node.destroy_node()
    # uninit
    rclpy.shutdown()

if __name__ == "__main__":
    main()
