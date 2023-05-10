#!/usr/bin/env python
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

if __name__ == '__main__':
   rospy.init_node("ultrasonic_sensor_to_pointcloud")
    cloud_pub = rospy.Publisher("pc_ultrasonic", PointCloud2, queue_size=4)
    rate = rospy.Rate(4)

    frame_id = rospy.get_param("frame_id", "fcu")
    offsets = rospy.get_param("offsets", [0, 0, 0, 0])

    header = std_msgs.msg.Header()
    header.frame_id = frame_id
    fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            #PointField('rgb', 12, PointField.UINT32, 1),
            PointField('rgb', 16, PointField.UINT32, 1),
        ]

    points = [
        [1, 1, 0, 0xFF], # blue
        [1, -1, 0, 0xFF00], # green
        [-1, 1, 0, 0xFF0000], # red
        [-1, -1, 0, 0x00FFFF], # cyan
        [0, 0, 1, 0xFFFF00] # yellow
    ]
    while not rospy.is_shutdown():
        header.stamp = rospy.Time.now()
        pc = point_cloud2.create_cloud(header, fields, points)
        cloud_pub.publish(pc)
        rate.sleep()

    rospy.spin()
