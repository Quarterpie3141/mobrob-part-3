#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import math

class MovingObjectTracker(Node):
    def __init__(self):
        super().__init__('moving_object_tracker')
        
        # Subscriptions & Publications
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_moving_objects', 10)
        
        # Configuration parameters
        self.cluster_tolerance = 0.3  # Max distance (meters) between points in the same object
        self.min_cluster_size = 3     # Min points to be considered a person/object
        self.max_cluster_size = 30    # Max points (prevents walls from being grouped)
        self.movement_threshold = 0.15 # Min distance (meters) an object must move to be "moving"
        
        self.prev_centroids = []       # Tracks centroids from the last frame

    def scan_callback(self, msg: LaserScan):
        # Step 1: Convert LaserScan Polar coordinates to Cartesian (X, Y)
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))
            angle += msg.angle_increment

        if not points:
            return

        # Step 2: Simple Euclidean Clustering (Point-to-Point distance check)
        clusters = []
        unvisited = set(range(len(points)))

        while unvisited:
            current_index = unvisited.pop()
            current_cluster = [points[current_index]]
            queue = [points[current_index]]

            while queue:
                curr_pt = queue.pop(0)
                neighbors = []
                for idx in list(unvisited):
                    pt = points[idx]
                    # Compute distance between points
                    dist = math.sqrt((curr_pt[0] - pt[0])**2 + (curr_pt[1] - pt[1])**2)
                    if dist < self.cluster_tolerance:
                        neighbors.append(idx)
                        current_cluster.append(pt)
                        queue.append(pt)
                for idx in neighbors:
                    unvisited.remove(idx)
            
            if self.min_cluster_size <= len(current_cluster) <= self.max_cluster_size:
                clusters.append(current_cluster)

        # Step 3: Compute Centroids (the center point of each cluster)
        current_centroids = []
        for cluster in clusters:
            avg_x = sum(pt[0] for pt in cluster) / len(cluster)
            avg_y = sum(pt[1] for pt in cluster) / len(cluster)
            current_centroids.append((avg_x, avg_y))

        # Step 4: Track movement against previous frame
        moving_objects = []
        if self.prev_centroids:
            for curr_c in current_centroids:
                # Find closest centroid in previous frame
                min_dist = float('inf')
                for prev_c in self.prev_centroids:
                    dist = math.sqrt((curr_c[0] - prev_c[0])**2 + (curr_c[1] - prev_c[1])**2)
                    if dist < min_dist:
                        min_dist = dist
                
                # If it matches an old item but moved greater than threshold, it's moving!
                if self.movement_threshold < min_dist < 1.5: 
                    moving_objects.append(curr_c)

        self.prev_centroids = current_centroids
        
        # Step 5: Publish Results to RViz
        self.publish_markers(moving_objects, msg.header.frame_id)

    def publish_markers(self, centroids, frame_id):
        marker_array = MarkerArray()
        
        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, (cx, cy) in enumerate(centroids):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "moving_people"
            marker.id = i
            marker.type = Marker.CYLINDER # Cylinder shape looks like a standing person
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = 0.0 # Ground plane level
            
            # Cylinder scale (human size width and height)
            marker.scale.x = 0.4 
            marker.scale.y = 0.4
            marker.scale.z = 1.2
            
            # Vibrant Red color for moving entities
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = MovingObjectTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()