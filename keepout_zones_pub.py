#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Header
from shapely.geometry import MultiPoint

class KeepoutPublisher(Node):
    def __init__(self):
        super().__init__('keepout_publisher')
        self.pub = self.create_publisher(PolygonStamped, '/keepout_polygons', 10)
        self.timer = self.create_timer(2.0, self.publish_zones)

    def publish_zones(self):
        # Hardcoded coordinates
        red_cones = [
            (0.0, 0.0),
            (0.0, 1.0),
            (1.0, 1.0),
            (1.0, 0.0)
        ]
        polygon_pts = MultiPoint(red_cones).convex_hull.exterior.coords[:-1]

        poly_msg = PolygonStamped()
        poly_msg.header = Header()
        poly_msg.header.frame_id = 'map'

        for x, y in polygon_pts:
            pt = Point32()
            pt.x, pt.y = float(x), float(y)
            pt.z = 0.0
            poly_msg.polygon.points.append(pt)

        self.pub.publish(poly_msg)
        self.get_logger().info(f"Published keepout polygon with {len(polygon_pts)} points")

def main():
    rclpy.init()
    node = KeepoutPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
