#! /usr/bin/env python3
# coding=utf-8
import rospy

import cv2
import numpy as np
# import torch
# from ultralytics import YOLO
from std_msgs.msg import Header 

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ars408_msg.msg import RadarPoints
import message_filters
from numpy.polynomial import polyutils as pu
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

class RadarPoint():
    def __init__(self, point_x=0, point_y=0, point_speed=0, point_dist=0):
        self.point_x = point_x
        self.point_y = point_y
        self.point_speed = point_speed
        self.point_dist = point_dist


def calculate_radar_points_averages(points):
    if len(points) == 1:
        return points[0]

    avg = RadarPoint()

    for p in points:
        avg.point_x += p.point_x
        avg.point_y += p.point_y
        avg.point_speed += p.point_speed
        avg.point_dist += p.point_dist
    
    avg.point_x /= len(points)
    avg.point_y /= len(points)
    avg.point_speed /= len(points)
    avg.point_dist /= len(points)

    return avg

import sys
class RadarRGBVisualizer():
    def __init__(self, use_time_synchronizer=True):
        if use_time_synchronizer:
            self.sub_image = message_filters.Subscriber("/camera/image_color", Image)
            self.sub_radar = message_filters.Subscriber("/radar/front_center/decoded_messages", RadarPoints)
            self.sub_lidar = message_filters.Subscriber("/ouster/points", PointCloud2)
            # self.sub_radar = message_filters.Subscriber("/radar/pointcloud", PointCloud2)
            self.radar_pointcloud_sync = rospy.Publisher("/radar/radar_pointcloud_sync", PointCloud2, queue_size=1)
            
            # self.sync = message_filters.ApproximateTimeSynchronizer([self.sub_image, self.sub_radar], 10, 0.2, reset=True)
            self.sync = message_filters.ApproximateTimeSynchronizer([self.sub_image, self.sub_radar, self.sub_lidar], 10, 0.2, reset=True)
            self.sync.registerCallback(self.callback)
        # else:
        #     self.sub_image = rospy.Subscriber("/camera/image_color", Image, self.image_cb, queue_size=1)
        #     self.sub_radar = rospy.Subscriber("/radar/front_center/decoded_messages", RadarPoints, self.radar_cb, queue_size=1)

        self.pub_fusion_image = rospy.Publisher("/radar_rgb/image", Image, queue_size=1)

        self.bridge = CvBridge()
        self.image = None
        self.radar_points = None

        # =============1208*720=============
        # self.cm_in = np.array([1743.491217, 0, 694.614295,
        #                        0, 1748.793251, 348.070293, 
        #                        0, 0, 1]).reshape((3, 3))
        
        self.cm_in = np.array([
            [1873.719785, 0, 896.573551],
            [0, 1406.828106, 535.307287], 
            [0, 0, 1]
        ], dtype=np.float32)

        # self.T = np.array([
        # [1, 0, 0, -1.88],#depth
        # [0, 1, 0, 1],#left right
        # [0, 0, 1, 1.4],#  上下頃角,左右頃角, 1, vertical
        # [0, 0, 0, 1]])

        self.T = np.array([
        [1, 0, 0, 1.5],#depth
        [0.13, 1, 0, 1],#left right
        [-0.15, 0, 1, 1.1],#  上下頃角,左右頃角, 1, vertical
        [0, 0, 0, 1]])

        self.cm_ex = np.array([0, 1, 0, 0, 
                               0, 0, -1, 0, 
                               -1, 0, 0, 0]).reshape((3, 4))


        self.rt = self.cm_ex @ self.T
        self.p = self.cm_in @ self.rt

        self.image_width, self.image_height = 1600,900
        print('ready into callback======================')

    
    def image_cb(self, image: Image):
        self.image = image
    
    def radar_cb(self, radar_points: RadarPoints):
        self.radar_points = radar_points
    
    def project_to_radar(self, points, projection_matrix):
        num_pts = points.shape[1]
        points = np.vstack([points, np.ones((1, num_pts))])
        points = np.dot(projection_matrix, points)
        return points[:3, :]

    def project_to_image(self, points, projection_matrix):
        num_pts = points.shape[1]
        points = np.vstack((points, np.ones((1, num_pts))))
        # print('projec: ', projection_matrix)
        # print('\n')
        points = projection_matrix @ points
        points[:2, :] /= points[2, :]
        # print("points == ",points[:2, :])
        return points[:2, :]
    
    # def callback(self, image, radar_points):
    #     print("======================camera & radar callback==============================================")
    #     self.image = image
    #     self.radar_points = radar_points
    #     image_sync = rospy.Publisher("/image/image_sync", Image, queue_size=1)
    #     image_sync.publish(self.image)
    #     radar_sync = rospy.Publisher("/radar/radar_sync", RadarPoints, queue_size=1)
    #     radar_sync.publish(radar_points)
    #     self.loop()

        
    # def callback(self, image, lidar_points):
    #     print("=============================camera & lidar callback=======================================")
    #     self.image = image
    #     imagerrrr = self.bridge.imgmsg_to_cv2(self.image)
    #     height, width, channels = imagerrrr.shape
    #     print("Image Size - Height: {}, Width: {}, Channels: {}".format(height, width, channels))
    #     image_sync = rospy.Publisher("/camera/image_sync", Image, queue_size=1)
    #     image_sync.publish(image)
    #     lidar_sync = rospy.Publisher("/lidar/lidar_sync",PointCloud2, queue_size=1)
    #     lidar_sync.publish(lidar_points)

    def callback(self, image, radar_points, lidar_points):
        print("==================================camera & radar & lidar callback==============================================")
        self.image = image
        self.radar_points = radar_points
        self.lidar_points = lidar_points
        image_sync = rospy.Publisher("/camera/image_sync", Image, queue_size=1)
        image_sync.publish(image)
        radar_sync = rospy.Publisher("/radar/radar_sync", RadarPoints, queue_size=1)
        radar_sync.publish(radar_points)
        lidar_sync = rospy.Publisher("/lidar/lidar_sync",PointCloud2, queue_size=1)
        lidar_sync.publish(lidar_points)

        # # Convert RadarPoints to PointCloud2
        points = []
        for point in radar_points.rps:
            # Assuming distX, distY, distZ are the coordinates of radar points
            # You may need to adjust this part based on the actual structure of RadarPoints
            points.append([point.id,point.dynProp,point.distX,point.distY,
                           point.vrelX,point.vrelY,point.rcs,point.prob,point.classT,point.angle,point.vrelY,point.height])
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"
        fields = [PointField(name="id", offset=0, datatype=PointField.INT32, count=1),
                  PointField(name="dynProp", offset=4, datatype=PointField.INT32, count=1),
                  PointField(name="distX", offset=8, datatype=PointField.FLOAT64, count=1),
                  PointField(name="distY", offset=16, datatype=PointField.FLOAT64, count=1),
                  PointField(name="vrelX", offset=24, datatype=PointField.FLOAT64, count=1),
                  PointField(name="vrelY", offset=28, datatype=PointField.FLOAT64, count=1),
                  PointField(name="rcs", offset=32, datatype=PointField.FLOAT64, count=1),
                  PointField(name="prob", offset=36, datatype=PointField.INT32, count=1),
                  PointField(name="classT", offset=40, datatype=PointField.INT32, count=1),
                  PointField(name="angle", offset=44, datatype=PointField.FLOAT64, count=1),
                  PointField(name="vrelY", offset=48, datatype=PointField.FLOAT64, count=1),
                  PointField(name="height", offset=52, datatype=PointField.FLOAT64, count=1),
                  ]

        cloud = pc2.create_cloud(header, fields, points)
        
        self.radar_pointcloud_sync.publish(cloud)
        print('ready into loop====================')
        self.loop()
    
    def loop(self):
        print('into loop===================')
        if not self.image or not self.radar_points:
            return
        
        radar_image = self.bridge.imgmsg_to_cv2(self.image)


        radar_image = np.clip(radar_image, 0, 255).astype(np.uint8) # test56

        radar_points = self.radar_points.rps
        # radar_points = self.radar_points.rcs
        points_3d = np.empty(shape=(0, 3))
        for p in radar_points:
            points_3d = np.append(points_3d, [[p.distX, p.distY, 1.0]], axis=0)
        
        points_2d = self.project_to_image(points_3d.transpose(), self.p)

        
        inds = np.where((points_2d[0, :] < self.image_width*0.9) & (points_2d[0, :] >= self.image_width*0.1)
                & (points_2d[1, :] < self.image_height) & (points_2d[1, :] >= 0)
                & (points_3d[:, 0] > 0)
                )[0]

        points_2d = points_2d[:, inds]
        points_3d = points_3d[inds, :]
        radar_points = [radar_points[i] for i in inds]
        for p in range(points_2d.shape[1]):
            depth = (250 - points_3d[p, 0]) / 250            
            point_x = int(points_2d[0, p] * 1)
            point_y = int(points_2d[1, p] * 1) # 0.7 
            # plot line or circle on RGB image 
            point_radius  = max(int(10 * depth), 4)
            # print("plot plot")
            cv2.circle(radar_image, (point_x, point_y),point_radius,(int(255 * abs(depth)),50,int(255 - 255 * abs(depth))),-1)

        radar_image_msg = self.bridge.cv2_to_imgmsg(radar_image)
        self.pub_fusion_image.publish(radar_image_msg)

if __name__ == "__main__":
    try:
        rospy.init_node("Radar RGB Visualizer")

        use_time_synchronizer = True
        v = RadarRGBVisualizer(use_time_synchronizer=use_time_synchronizer)

        if use_time_synchronizer:
            rospy.spin()
        else:
            r = rospy.Rate(60)
            while not rospy.is_shutdown():
                v.loop()
                r.sleep()

    except rospy.ROSException as e:
        rospy.logerr(str(e))