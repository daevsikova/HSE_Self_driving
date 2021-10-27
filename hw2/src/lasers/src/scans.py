#! /usr/bin/python3

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class LaserProcesser:
    def __init__(self):
        self.cell_size = 0.1

        self.marker_pub = rospy.Publisher('/marker_topic', Marker, queue_size=1)
        self.map_pub = rospy.Publisher('/map_topic', OccupancyGrid, queue_size=1)

    def get_and_filter_points(self, msg):
        self.ranges = np.array(msg.ranges)
        
        mask = np.zeros_like(self.ranges).astype(bool)
        for i in range(len(self.ranges)-4):
            x1, x2, x3, x4, x5 = self.ranges[i:i+5]
            mean = np.mean([x1, x2, x4, x5])
            if np.abs(x3 - mean) < 0.05:
                mask[i + 2] = True

        mask[0] = mask[-1] = mask[1] = mask[-2] = True
        print(f'Total num points: {mask.shape[0]}')
        print(f'Num points after filtering: {mask.sum()}')
        
        angles = msg.angle_min + np.arange(self.ranges.shape[0]) * msg.angle_increment
        self.x = (self.ranges * np.cos(angles))
        self.y = (self.ranges * np.sin(angles))
        # [mask]
        
    def visualize_points(self):
        marker = Marker()
        marker.header.frame_id = "base_laser_link"
        marker.action = 0

        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.scale.x = self.cell_size
        marker.scale.y = self.cell_size
        marker.scale.z = self.cell_size
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        marker.type = marker.POINTS
        points = []
        for x, y in zip(self.x, self.y):
            points.append(Point(x, y, 0.4))
        marker.points = points
        
        self.marker_pub.publish(marker)
        
    def visualize_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'base_laser_link'
        grid.info.resolution = self.cell_size
        R = 10
        w = int(np.ceil(R / self.cell_size * 2))
        grid.info.width = w
        grid.info.height = w
        grid.info.origin.position.x = -R
        grid.info.origin.position.y = -R
        grid.info.origin.position.z = 0.0

        data = np.zeros(shape=(w, w)).astype(int)
        
        for x, y in zip(self.x, self.y):
           
           if x**2 + y**2 <= R**2:
                i = int((x + R) / self.cell_size)
                j = int((y + R) / self.cell_size)
                data[j, i] = 100

        grid.data = data.reshape(-1)
        self.map_pub.publish(grid)

    def process(self, msg):
        self.get_and_filter_points(msg)
        self.visualize_points()
        self.visualize_map()


if __name__ == '__main__':
    rospy.init_node('visualization')
    rospy.Subscriber('/base_scan', LaserScan, LaserProcesser().process)
    rospy.spin()
