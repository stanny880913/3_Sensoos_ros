#! /usr/bin/env python3
# coding=utf-8
import rospy

import cv2
import numpy as np
# import torch
# from ultralytics import YOLO
from std_msgs.msg import Header, ColorRGBA

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
            print('into use_time_synchronizer!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self.sub_image = message_filters.Subscriber("/camera/image_color", Image)
            self.sub_radar = message_filters.Subscriber("/radar/front_center/decoded_messages", RadarPoints)
            # self.sub_radar = message_filters.Subscriber("/radar/pointcloud", PointCloud2)
            self.radar_pointcloud_sync = rospy.Publisher("/radar/radar_pointcloud_sync", PointCloud2, queue_size=1)
            self.sync = message_filters.ApproximateTimeSynchronizer([self.sub_image, self.sub_radar], 10, 0.2, reset=True)
            self.sync.registerCallback(self.callback)
        else:
            self.sub_image = rospy.Subscriber("/camera/image_color", Image, self.image_cb, queue_size=1)
            self.sub_radar = rospy.Subscriber("/radar/front_center/decoded_messages", RadarPoints, self.radar_cb, queue_size=1)

        self.pub_fusion_image = rospy.Publisher("/radar_rgb/image", Image, queue_size=1)

        self.bridge = CvBridge()
        self.image = None
        self.radar_points = None


        """
        camera matrix
        [2033,    0, 1068]
        [0   , 2056,  539]
        [0   ,    0,    1]
        """
        # self.p = np.array([1068, -2033, 0, 0, 539, 0, -2056, 0, 1, 0, 0, 0]).reshape((3, 4))
        # self.p = np.array([1127.134065, -1757.807569, 0, 0, 833.097400, 0, -1762.138722, 0, 1, 0, 0, 0]).reshape((3, 4))
        self.cm_in = np.array([1757.807569, 0, 1127.134065,
                               0, 1762.138722, 833.097400, 
                               0, 0, 1]).reshape((3, 3))
        self.T = np.array([
            [1, 0, 0, 1.88],#depth
            [0, 1, 0, 0],#left right
            [-0.06, 0, 0.25, 1.15],#  上下頃角,左右頃角, 1, vertical
            [0, 0, 0, 1]])
        #       camera  radar
        # height 173    58 =115
        # vertical 188
        self.cm_ex = np.array([0, 1, 0, 0, 
                               0, 0, -1, 0, 
                               -1, 0, 0, 0]).reshape((3, 4))


        self.rt = self.cm_ex @ self.T
        self.p = self.cm_in @ self.rt
        # print('=========================================================')
        # print(self.p)
        self.image_width, self.image_height = 2048, 1536

    
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
        print('projec: ', projection_matrix)
        print('\n')
        points = projection_matrix @ points
        points[:2, :] /= points[2, :]
        print("points == ",points[:2, :])
        return points[:2, :]
    
    def callback(self, image, radar_points):
        self.image = image
        self.radar_points = radar_points
        image_sync = rospy.Publisher("/image/image_sync", Image, queue_size=1)
        image_sync.publish(image)
        radar_sync = rospy.Publisher("/radar/radar_sync", RadarPoints, queue_size=1)
        radar_sync.publish(radar_points)

        # Convert RadarPoints to PointCloud2
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
                  PointField(name="dynProp", offset=8, datatype=PointField.INT32, count=1),
                  PointField(name="distX", offset=16, datatype=PointField.FLOAT64, count=1),
                  PointField(name="distY", offset=24, datatype=PointField.FLOAT64, count=1),
                  PointField(name="vrelX", offset=32, datatype=PointField.FLOAT64, count=1),
                  PointField(name="vrelY", offset=40, datatype=PointField.FLOAT64, count=1),
                  PointField(name="rcs", offset=48, datatype=PointField.FLOAT64, count=1),
                  PointField(name="prob", offset=64, datatype=PointField.INT32, count=1),
                  PointField(name="classT", offset=72, datatype=PointField.INT32, count=1),
                  PointField(name="angle", offset=80, datatype=PointField.FLOAT64, count=1),
                  PointField(name="vrelY", offset=88, datatype=PointField.FLOAT64, count=1),
                  PointField(name="height", offset=96, datatype=PointField.FLOAT64, count=1),
                  ]

        cloud = pc2.create_cloud(header, fields, points)
        
        self.radar_pointcloud_sync.publish(cloud)

        self.loop()
    
    def loop(self):
        if not self.image or not self.radar_points:
            return
        
        radar_image = self.bridge.imgmsg_to_cv2(self.image)
        radar_image = np.clip(radar_image, 0, 255).astype(np.uint8) # test56
        radar_points = self.radar_points.rps
        # radar_points = self.radar_points.rcs
        points_3d = np.empty(shape=(0, 3))
        for p in radar_points:
            points_3d = np.append(points_3d, [[p.distX, p.distY, 1.0]], axis=0)
        
        # points_2d = self.project_to_radar(points_3d.transpose(), self.p)
        points_2d = self.project_to_image(points_3d.transpose(), self.p)

        scale_coordinate = False
        if scale_coordinate:
            points_2d[0, :] = points_2d[0, :] / 1280 * 2048
            points_2d[1, :] = points_2d[1, :] / 720 * 1536
        
        inds = np.where((points_2d[0, :] < self.image_width*0.9) & (points_2d[0, :] >= self.image_width*0.1)
                & (points_2d[1, :] < self.image_height) & (points_2d[1, :] >= 0)
                & (points_3d[:, 0] > 0)
                )[0]

        points_2d = points_2d[:, inds]
        points_3d = points_3d[inds, :]
        radar_points = [radar_points[i] for i in inds]

        # offset_x, offset_y = 100, 0
        # radar_image = radar_image[:, offset_x:-offset_x]
        # radar_image = cv2.resize(radar_image, (self.image_width, self.image_height))
        # rospy.loginfo_once(radar_image.shape)

        # rospy.loginfo_once((radar_image.shape))

        # print_radar_info = True
        # print_box_radar_mapping = False
        # box_ids = []

        # scale_x_old_domain = (0, 2047)
        # scale_x_new_domain = (0.75, 1.25)
        # scale_x_new_domain = (1, 1)
        # scale_y_old_domain = (15, 60)
        # scale_y_new_domain = (0.8, 3)
        # scale_y_new_domain = (1, 1)
        for p in range(points_2d.shape[1]):
            depth = (250 - points_3d[p, 0]) / 250
            
            
            point_x = int(points_2d[0, p] * 1)
            point_y = int(points_2d[1, p] * 1) # 0.7 

            # point_speed = np.sqrt(pow(radar_points[p].vrelX, 2) + pow(radar_points[p].vrelY, 2)) # radar point speed in m/s
            # point_dist = np.sqrt(pow(radar_points[p].distX, 2) + pow(radar_points[p].distY, 2))
            # 0: not post process
            # 1: Scene C (mcdo)
            # 2: Scene B (chicken)
            # 3: Scene A (elementary school)
            # hard_postprocess = 1
            # image_width = radar_image.shape[1]
            # image_height = radar_image.shape[0]
            # if hard_postprocess == 1:
            #     # if point_y < image_height / 2:
            #     if hard_postprocess == 1:
            #         old_point_y = point_y
            #         scale_y = pu.mapdomain(point_dist, scale_y_old_domain, scale_y_new_domain)
            #         point_y = int(image_height - point_y * scale_y)
            #         scale_x = pu.mapdomain(point_x, scale_x_old_domain, scale_x_new_domain)
            #         if scale_x > 1:
            #             scale_x = max(scale_x, 1.1)
            #         else:
            #             scale_x = min(0.9, scale_x)
            #         point_x = int(point_x * scale_x)



            # if p_speed > 1 or radar_points[p].distX < 10:
            # if (radar_points[p].classT != 7 and point_dist < 20) and point_speed > 0.5:
            # if point_x > 500 and point_dist > 15 and point_dist < 50 and point_speed > 1:
                # check if inside bounding box
                # box_id = None
                # for i, (x, y, w, h) in enumerate(preds.boxes.xywh):
                #     box_ids.append(i)
                #     if point_x > (x - w / 1.7) and point_x < (x + w / 1.7) \
                #         and point_y > (y - h / 1.7) and point_y < (y + h / 1.7):
                #         box_radar_mapping[i].append(RadarPoint(point_x, point_y, point_speed, point_dist))

                # if print_radar_info:
                #     cv2.putText(radar_image, f"{point_speed * 3.6:.1f} km/h", (point_x, point_y), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)
                #     cv2.putText(radar_image, f"{point_dist:.1f} m", (point_x, point_y+50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)
                #     # cv2.putText(radar_image, f"{radar_points[p].classT}", (point_x, point_y+100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4, cv2.LINE_AA)
            
            # radar point y-axis is opposite
            # length = max(int(80 * depth), 40)
            # thickness = max(int(9*depth), 3)
            # cv2.line(radar_image, (point_x, 1536-point_y+length), (point_x, 1536-point_y), (0, int(255 * abs(depth)), 50), thickness=thickness)
            
            # plot line or circle on RGB image 
            point_radius = length = max(int(20 * depth), 5)
            cv2.circle(radar_image, (point_x, point_y),point_radius,(int(255 * abs(depth)),50,int(255 - 255 * abs(depth))),-1)

            # length = int(max(int(100 * depth), 40)/2)
            # thickness = max(int(9*depth), 3)
            # cv2.line(radar_image, (point_x, point_y+length), (point_x, point_y-length), (int(255 * abs(depth)),int(255 - 255 * abs(depth))), thickness=thickness)


        # if print_box_radar_mapping:
        #     for i, (x, y, w, h) in enumerate(preds.boxes.xywh):
        #         cv2.rectangle(radar_image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (0, 255, 0), 2)
        #         cls = preds.names[int(preds.boxes.cls[i])]
            
        #         if len(box_radar_mapping[i]):
        #             avg = calculate_radar_points_averages(box_radar_mapping[i])
        #             cv2.putText(radar_image, f"{cls} {avg.point_dist:.1f}m {avg.point_speed * 3.6:.1f}km/h", (int(x - w / 2), int(y - h / 2)-15), cv2.FONT_HERSHEY_SIMPLEX, .9, (0, 255, 0), 2, cv2.LINE_AA)


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



# #! /usr/bin/env python3
# # coding=utf-8
# import rospy

# import cv2
# import numpy as np
# # import torch
# # from ultralytics import YOLO

# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from ars408_msg.msg import RadarPoints
# import message_filters
# from numpy.polynomial import polyutils as pu


# class RadarPoint():
#     def __init__(self, point_x=0, point_y=0, point_speed=0, point_dist=0):
#         self.point_x = point_x
#         self.point_y = point_y
#         self.point_speed = point_speed
#         self.point_dist = point_dist


# def calculate_radar_points_averages(points):
#     if len(points) == 1:
#         return points[0]

#     avg = RadarPoint()

#     for p in points:
#         avg.point_x += p.point_x
#         avg.point_y += p.point_y
#         avg.point_speed += p.point_speed
#         avg.point_dist += p.point_dist
    
#     avg.point_x /= len(points)
#     avg.point_y /= len(points)
#     avg.point_speed /= len(points)
#     avg.point_dist /= len(points)

#     return avg

# import sys
# class RadarRGBVisualizer():
#     def __init__(self, use_time_synchronizer=True):
#         if use_time_synchronizer:
#             self.sub_image = message_filters.Subscriber("/camera/image_color", Image)
#             self.sub_radar = message_filters.Subscriber("/radar/front_center/decoded_messages", RadarPoints)

#             self.sync = message_filters.ApproximateTimeSynchronizer([self.sub_image, self.sub_radar], 10, 0.2, reset=True)
#             self.sync.registerCallback(self.callback)
#         else:
#             self.sub_image = rospy.Subscriber("/camera/image_color", Image, self.image_cb, queue_size=1)
#             self.sub_radar = rospy.Subscriber("/radar/front_center/decoded_messages", RadarPoints, self.radar_cb, queue_size=1)

#         self.pub_fusion_image = rospy.Publisher("/radar_rgb/image", Image, queue_size=1)

#         self.bridge = CvBridge()
#         self.image = None
#         self.radar_points = None


#         """
#         camera matrix
#         [2033,    0, 1068]
#         [0   , 2056,  539]
#         [0   ,    0,    1]
#         """
#         # self.p = np.array([1068, -2033, 0, 0, 539, 0, -2056, 0, 1, 0, 0, 0]).reshape((3, 4))
#         # self.p = np.array([1127.134065, -1757.807569, 0, 0, 833.097400, 0, -1762.138722, 0, 1, 0, 0, 0]).reshape((3, 4))
#         self.cm_in = np.array([1757.807569, 0, 1127.134065, 0, 1762.138722, 833.097400, 0, 0, 1]).reshape((3, 3))
#         self.T = np.array([
#             [1, 0, 0, 0],
#             [0, 1, 0, -1.88],
#             [0, 0, 1, 0],
#             [0, 0, 0, 1]
#         ]) # x, y, z
#         #       camera  radar
#         # height 173    58 =115
#         # vertical 188
#         self.cm_ex = np.array([0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0]).reshape((3, 4))
#         self.rt = self.cm_ex @ self.T
#         self.p = self.cm_in @ self.rt
#         # print('=========================================================')
#         # print(self.p)
#         """
#         camera matrix
#         [2033,    0, 1068]  
#         [0   , 2056,  539]
#         [0   ,    0,    1]
#         """
#         self.image_width, self.image_height = 2048, 1536

    
#     def image_cb(self, image: Image):
#         self.image = image
    
#     def radar_cb(self, radar_points: RadarPoints):
#         self.radar_points = radar_points
    
#     def project_to_radar(self, points, projection_matrix):
#         num_pts = points.shape[1]
#         points = np.vstack([points, np.ones((1, num_pts))])
#         points = np.dot(projection_matrix, points)
#         return points[:3, :]

#     def project_to_image(self, points, projection_matrix):
#         num_pts = points.shape[1]
#         points = np.vstack((points, np.ones((1, num_pts))))
#         print('projec: ', projection_matrix)
#         print('\n')
#         points = projection_matrix @ points
#         points[:2, :] /= points[2, :]
#         print("points == ",points[:2, :])
#         return points[:2, :]
    
#     def callback(self, image, radar_points):
#         self.image = image
#         self.radar_points = radar_points
#         self.loop()
    
#     def loop(self):
#         if not self.image or not self.radar_points:
#             return
        
#         radar_image = self.bridge.imgmsg_to_cv2(self.image)
        
#         radar_points = self.radar_points.rps
#         points_3d = np.empty(shape=(0, 3))
#         for p in radar_points:
#             points_3d = np.append(points_3d, [[p.distX, p.distY, 1.0]], axis=0)
        
#         # points_2d = self.project_to_radar(points_3d.transpose(), self.p)
#         points_2d = self.project_to_image(points_3d.transpose(), self.p)

#         scale_coordinate = False
#         if scale_coordinate:
#             points_2d[0, :] = points_2d[0, :] / 1280 * 2048
#             points_2d[1, :] = points_2d[1, :] / 720 * 1536
        
#         inds = np.where((points_2d[0, :] < self.image_width*0.9) & (points_2d[0, :] >= self.image_width*0.1)
#                 & (points_2d[1, :] < self.image_height) & (points_2d[1, :] >= 0)
#                 & (points_3d[:, 0] > 0)
#                 )[0]

#         points_2d = points_2d[:, inds]
#         points_3d = points_3d[inds, :]
#         radar_points = [radar_points[i] for i in inds]

#         # offset_x, offset_y = 100, 0
#         # radar_image = radar_image[:, offset_x:-offset_x]
#         # radar_image = cv2.resize(radar_image, (self.image_width, self.image_height))
#         # rospy.loginfo_once(radar_image.shape)

#         # rospy.loginfo_once((radar_image.shape))

#         # print_radar_info = True
#         # print_box_radar_mapping = False
#         # box_ids = []

#         # scale_x_old_domain = (0, 2047)
#         # scale_x_new_domain = (0.75, 1.25)
#         # scale_x_new_domain = (1, 1)
#         # scale_y_old_domain = (15, 60)
#         # scale_y_new_domain = (0.8, 3)
#         # scale_y_new_domain = (1, 1)
#         for p in range(points_2d.shape[1]):
#             depth = (250 - points_3d[p, 0]) / 250
            
            
#             point_x = int(points_2d[0, p] * 1)
#             point_y = int(points_2d[1, p] * 1) # 0.7 

#             # point_speed = np.sqrt(pow(radar_points[p].vrelX, 2) + pow(radar_points[p].vrelY, 2)) # radar point speed in m/s
#             # point_dist = np.sqrt(pow(radar_points[p].distX, 2) + pow(radar_points[p].distY, 2))
#             # 0: not post process
#             # 1: Scene C (mcdo)
#             # 2: Scene B (chicken)
#             # 3: Scene A (elementary school)
#             # hard_postprocess = 1
#             # image_width = radar_image.shape[1]
#             # image_height = radar_image.shape[0]
#             # if hard_postprocess == 1:
#             #     # if point_y < image_height / 2:
#             #     if hard_postprocess == 1:
#             #         old_point_y = point_y
#             #         scale_y = pu.mapdomain(point_dist, scale_y_old_domain, scale_y_new_domain)
#             #         point_y = int(image_height - point_y * scale_y)
#             #         scale_x = pu.mapdomain(point_x, scale_x_old_domain, scale_x_new_domain)
#             #         if scale_x > 1:
#             #             scale_x = max(scale_x, 1.1)
#             #         else:
#             #             scale_x = min(0.9, scale_x)
#             #         point_x = int(point_x * scale_x)



#             # if p_speed > 1 or radar_points[p].distX < 10:
#             # if (radar_points[p].classT != 7 and point_dist < 20) and point_speed > 0.5:
#             # if point_x > 500 and point_dist > 15 and point_dist < 50 and point_speed > 1:
#                 # check if inside bounding box
#                 # box_id = None
#                 # for i, (x, y, w, h) in enumerate(preds.boxes.xywh):
#                 #     box_ids.append(i)
#                 #     if point_x > (x - w / 1.7) and point_x < (x + w / 1.7) \
#                 #         and point_y > (y - h / 1.7) and point_y < (y + h / 1.7):
#                 #         box_radar_mapping[i].append(RadarPoint(point_x, point_y, point_speed, point_dist))

#                 # if print_radar_info:
#                 #     cv2.putText(radar_image, f"{point_speed * 3.6:.1f} km/h", (point_x, point_y), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)
#                 #     cv2.putText(radar_image, f"{point_dist:.1f} m", (point_x, point_y+50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2, cv2.LINE_AA)
#                 #     # cv2.putText(radar_image, f"{radar_points[p].classT}", (point_x, point_y+100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4, cv2.LINE_AA)
            
#             # radar point y-axis is opposite
#             length = max(int(80 * depth), 40)
#             thickness = max(int(9*depth), 3)
#             cv2.line(radar_image, (point_x, 1536-point_y+length), (point_x, 1536-point_y), (0, int(255 * abs(depth)), 50), thickness=thickness)
            
#             # plot line or circle on RGB image 
#             # point_radius = length = max(int(30 * depth), 15)
#             # cv2.circle(radar_image, (point_x, point_y),point_radius,(0,int(255 * abs(depth)),int(255 - 255 * abs(depth))),-1)

#             # length = max(int(250 * depth), 40)/2
#             # cv2.line(radar_image, (point_x, point_y+length), (point_x, point_y-length), (0,int(255 * abs(depth)),int(255 - 255 * abs(depth))), thickness=6)


#                 # if box_id != None:
#                 #     x,y,w,h = preds.boxes.xywh[box_id]
#                 #     cls = preds.names[int(preds.boxes.cls[box_id])]
#                 #     # rospy.loginfo(((x, y), (x+w, y+h)))
#                 #     cv2.putText(radar_image, f"{cls} {point_dist:.1f} m {point_speed * 3.6:.1f} km/h", (int(x - w / 2), int(y - h / 2)-25), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 4, cv2.LINE_AA)
#                 #     cv2.rectangle(radar_image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (0, 255, 0), 3)
                    
#             # if print_radar_info:
#             #     cv2.line(radar_image, (point_x, point_y+length), (point_x, point_y), (0, int(255 * abs(depth)), 50), thickness=6)

#         # if print_box_radar_mapping:
#         #     for i, (x, y, w, h) in enumerate(preds.boxes.xywh):
#         #         cv2.rectangle(radar_image, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)), (0, 255, 0), 2)
#         #         cls = preds.names[int(preds.boxes.cls[i])]
            
#         #         if len(box_radar_mapping[i]):
#         #             avg = calculate_radar_points_averages(box_radar_mapping[i])
#         #             cv2.putText(radar_image, f"{cls} {avg.point_dist:.1f}m {avg.point_speed * 3.6:.1f}km/h", (int(x - w / 2), int(y - h / 2)-15), cv2.FONT_HERSHEY_SIMPLEX, .9, (0, 255, 0), 2, cv2.LINE_AA)


#         radar_image_msg = self.bridge.cv2_to_imgmsg(radar_image)
#         self.pub_fusion_image.publish(radar_image_msg)

# if __name__ == "__main__":
#     try:
#         rospy.init_node("Radar RGB Visualizer")

#         use_time_synchronizer = True
#         v = RadarRGBVisualizer(use_time_synchronizer=use_time_synchronizer)

#         if use_time_synchronizer:
#             rospy.spin()
#         else:
#             r = rospy.Rate(60)
#             while not rospy.is_shutdown():
#                 v.loop()
#                 r.sleep()

#     except rospy.ROSException as e:
#         rospy.logerr(str(e))