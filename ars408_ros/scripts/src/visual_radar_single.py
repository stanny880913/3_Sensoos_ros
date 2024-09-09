#! /usr/bin/env python3
# coding=utf-8
import math
import rospy

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from ars408_msg.msg import RadarPoints


class RadarVisualizer():
    def __init__(self):
        self.sub_radar = rospy.Subscriber("/radar/front_center/decoded_messages", RadarPoints, self.radar_callback, queue_size=1)
        self.pub_marker = rospy.Publisher("/radar/marker", MarkerArray, queue_size=1)
        self.pub_range = rospy.Publisher("/radar/range", MarkerArray, queue_size=1)
        self.pub_radarpoint = rospy.Publisher("/radar/pointcloud", PointCloud2, queue_size=1)

    def radar_callback(self, radar_points: RadarPoints):
        markers = MarkerArray()

        # # clear previous markers
        markers.markers.append(Marker(
            header=Header(frame_id="base_link", stamp=rospy.Time.now()),
            action=Marker.DELETEALL
        ))

        # # radar points
        id = 0
        for i in radar_points.rps:
            marker = Marker(
                header=Header(frame_id="base_link", stamp=rospy.Time.now()),
                id=id,
                ns="front_center",
                type=Marker.CYLINDER,
                action=Marker.ADD,
                pose=Pose(
                    position=Point(x=i.distX, y=i.distY, z=1.0),
                    orientation=Quaternion(x=0, y=0, z=1)
                ),
                scale=Vector3(x=1, y=1, z=1.5),
                color=ColorRGBA(r=0.0, g=0.0, b=0.9, a=1.0)
            )
            markers.markers.append(marker)
            id = id + 1

        self.pub_marker.publish(markers)

        # radar range
        range_markers = MarkerArray()

        # ego arrow
        range_markers.markers.append(
            Marker(
                header=Header(frame_id="base_link", stamp=rospy.Time.now()),
                id=0,
                ns="ego_arrow",
                type=Marker.ARROW,
                action=Marker.ADD,
                scale=Vector3(x=5, y=0.5, z=0.5),
                color=ColorRGBA(r=1.0, b=0.5, g=0.0, a=1.0)
            )
        )

        i = 0
        for t in [[0, 0, 0]]:
            radar_transform = t
            range_marker = Marker(
                header=Header(frame_id="base_link", stamp=rospy.Time.now()),
                id=id,
                ns="range_marker_wide",
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                pose=Pose(
                    position=Point(x=0, y=0, z=0.1),
                    orientation=Quaternion(x=0, y=0, z=0, w=1.0)
                ),
                scale=Vector3(x=0.5, y=0.1, z=0.1),
                color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            )
            id = id + 1
            # p = Point(z=0.5)
            # range_marker.points.append(Point(x=0, y=0, z=0.5))
            # range_marker.points.append(Point(x=1, y=0, z=0.5))
            # range_marker.points.append(Point(x=2, y=0, z=0.5))
            # range_marker.points.append(Point(x=2, y=1, z=0.5))
            # range_marker.points.append(Point(x=2, y=2, z=0.5))
            # range_marker.points.append(Point(x=0, y=0, z=0.5))

            # wide range
            rotate = -40 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 70 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 70 + math.cos(rotate) * 0
            ))
            rotate = -46 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 35 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 35 + math.cos(rotate) * 0
            ))
            range_marker.points.append(Point(
                x=0 + radar_transform[0],
                y=0 + radar_transform[1]
            ))
            rotate = 46 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 35 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 35 + math.cos(rotate) * 0
            ))
            rotate = 40 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 70 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 70 + math.cos(rotate) * 0
            ))
            for i in range(40, -41, -5):
                rotate = i * math.pi / 180.0 + radar_transform[2]
                range_marker.points.append(Point(
                    x=math.cos(rotate) * 70 - math.sin(rotate) * 0,
                    y=math.sin(rotate) * 70 + math.cos(rotate) * 0
                ))
            range_markers.markers.append(range_marker)

            # narrow range
            range_marker = Marker(
                header=Header(frame_id="base_link", stamp=rospy.Time.now()),
                id=id,
                ns="range_marker_narrow",
                type=Marker.LINE_STRIP,
                action=Marker.ADD,
                pose=Pose(
                    position=Point(x=0, y=0, z=0.1),
                    orientation=Quaternion(x=0, y=0, z=0, w=1.0)
                ),
                scale=Vector3(x=0.5, y=0.1, z=0.1),
                color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            )
            id = id + 1
            
            rotate = 4 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 250 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 250 + math.cos(rotate) * 0
            ))
            rotate = 9 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 150 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 150 + math.cos(rotate) * 0
            ))
            range_marker.points.append(Point(
                x=0 + radar_transform[0],
                y=0 + radar_transform[1]
            ))
            rotate = -9 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 150 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 150 + math.cos(rotate) * 0
            ))
            rotate = -4 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 250 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 250 + math.cos(rotate) * 0
            ))
            rotate = 4 * math.pi / 180.0 + radar_transform[2]
            range_marker.points.append(Point(
                x=math.cos(rotate) * 250 - math.sin(rotate) * 0,
                y=math.sin(rotate) * 250 + math.cos(rotate) * 0
            ))
            range_markers.markers.append(range_marker)
        self.pub_range.publish(range_markers)

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
        self.pub_radarpoint.publish(cloud)

def main():
    try:
        rospy.init_node("Radar_Visualize")
        _ = RadarVisualizer()
        rospy.spin()
    except Exception as e:
        print(str(e))

if __name__ == "__main__":
    try:    
        main()
    except rospy.ROSException:
        pass
