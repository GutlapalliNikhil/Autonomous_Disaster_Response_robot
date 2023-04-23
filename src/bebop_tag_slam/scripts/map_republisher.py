#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid

# Thresholds for classifying grid cells
OBSTACLE_THRESHOLD = 75
UNKNOWN_THRESHOLD = 50

class MapProcessor:
    def __init__(self):
        rospy.init_node('map_cartographer', anonymous=True)
        self.sub = rospy.Subscriber('/map_updater', OccupancyGrid, self.callback)
        self.pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

    def process_map_data(self, data, width, height):
        """Process map data by classifying each grid cell as obstacle, free, or unknown."""
        processed_data = []
        for y in range(height):
            for x in range(width):
                i = (y * width) + x
                cell_value = data[i]
                if cell_value >= OBSTACLE_THRESHOLD:
                    processed_data.append(100)
                elif 0 <= cell_value < UNKNOWN_THRESHOLD:
                    processed_data.append(0)
                else:
                    processed_data.append(-1)
        return processed_data

    def callback(self, map_carto: OccupancyGrid):
        """Callback function for processing and republishing the occupancy grid."""
        processed_data = self.process_map_data(map_carto.data, map_carto.info.width, map_carto.info.height)
        map_carto.data = tuple(processed_data)
        self.pub.publish(map_carto)

    def run(self):
        """Starts the ROS node and spins until shutdown."""
        rospy.spin()

if __name__ == '__main__':
    map_processor = MapProcessor()
    map_processor.run()
