#!/usr/bin/env python3
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
from smbus2 import SMBus
import struct

if __name__ == '__main__':
    addr = 0x8 # bus address
    bus = SMBus(1) # indicates /dev/ic2-1

    rospy.init_node("ultrasonic_sensor_publisher")
    cloud_pub = rospy.Publisher("ultra_pc", PointCloud2, queue_size=4)
    rate = rospy.Rate(4)

    frame_id = rospy.get_param("frame_id", "fcu")
    offsets = rospy.get_param("offsets", [10, 5, 10, 0])
    orientations = rospy.get_param("orientations", [1, 6, 1, 5])

    header = std_msgs.msg.Header()
    header.frame_id = frame_id
    fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            #PointField('rgb', 12, PointField.UINT32, 1),
            PointField('rgb', 16, PointField.UINT32, 1),
        ]

    while not rospy.is_shutdown():
        try:
            req = bus.read_i2c_block_data(addr, ord('a'), 16) # get all sensors
            r = struct.iter_unpack('<f', bytearray(req))
            points = []
            c = 0
            for i in r:
                if i[0] < 300:
                    if orientations[c] == 1:
                        points.append([(i[0] + offsets[c]) / 100., 0, 0, 0xFFFF])
                    elif orientations[c] == 2:
                        points.append([(-i[0] - offsets[c]) / 100., 0, 0, 0xFFFF])
                    elif orientations[c] == 3:
                        points.append([0, (i[0] + offsets[c]) / 100., 0, 0xFFFF])
                    elif orientations[c] == 4:
                        points.append([0, (-i[0] - offsets[c]) / 100., 0, 0xFFFF])
                    elif orientations[c] == 5:
                        points.append([0, 0, (i[0] + offsets[c]) / 100., 0xFFFF])
                    elif orientations[c] == 6:
                        points.append([0, 0, (-i[0] - offsets[c]) / 100., 0xFFFF])
                c = c + 1
            header.stamp = rospy.Time.now()
            pc = point_cloud2.create_cloud(header, fields, points)
            cloud_pub.publish(pc)
            rate.sleep()
        except:
            pass
    rospy.spin()
