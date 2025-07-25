# ROS2-Humble: Data Publisher Script
#!/usr/bin/env python3

"""

This script connects to the CARLA simulator and collects live vehicle telemetry data
including steering angle and lateral offset from the lane center. The data is processed
to identify the vehicle's directional deviation (left, right, or center) and is published
at 20 Hz. The data is also logged in JSON format to support driver drowsiness detection
via downstream feature extraction.
"""

import rclpy
from rclpy.node import Node
import carla
import math
import json
import os

class DataPublisher(Node):
    """
    ROS 2 Node that interfaces with CARLA to extract and log steering and lane deviation data.
    """

    def __init__(self):
        super().__init__('data_publisher')
        self.get_logger().info('Initializing Data Publisher...')

        # Connect to CARLA simulator server
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        blueprint_library = self.world.get_blueprint_library()
        self.vehicle = None

        # Look for the first vehicle in the simulation
        for actor in self.world.get_actors():
            if 'vehicle' in actor.type_id:
                self.vehicle = actor
                break

        if self.vehicle is None:
            self.get_logger().error("No vehicle found in simulation!")
            return
        else:
            self.get_logger().info(f"Connected to vehicle: {self.vehicle.type_id}")

        # Retrieve the road map
        self.map = self.world.get_map()

        # Prepare for JSON logging
        self.data_log = []
        self.json_file_path = os.path.expanduser('~/Documents/carla_data_log.json')

        # Set timer to publish data every 0.05 seconds (20 Hz)
        self.create_timer(0.05, self.publisher_callback)

    def publisher_callback(self):
        """
        Callback function that collects vehicle data, calculates lateral offset,
        infers driving direction, and logs the results to JSON.
        """
        try:
            # Get current vehicle location and steering angle
            transform = self.vehicle.get_transform()
            location = transform.location
            vehicle_pos = (location.x, location.y)

            control = self.vehicle.get_control()
            steering_angle = control.steer  # Range: -1.0 (left) to +1.0 (right)

            # Get lane center from CARLA waypoint API
            waypoint = self.map.get_waypoint(location, project_to_road=True)
            lane_pos = waypoint.transform.location
            lane_center = (lane_pos.x, lane_pos.y)

            # Calculate Euclidean distance between vehicle and lane center
            dx = vehicle_pos[0] - lane_center[0]
            dy = vehicle_pos[1] - lane_center[1]
            lateral_offset = math.sqrt(dx**2 + dy**2)

            # Determine side (left/right/center) and apply sign
            if dx < -0.1:
                lateral_offset *= -1
                direction = 'left'
            elif dx > 0.1:
                direction = 'right'
            else:
                lateral_offset = 0.0
                direction = 'center'

            # Create data dictionary
            log_entry = {
                'steering_angle': round(steering_angle, 3),
                'lateral_offset_m': round(lateral_offset, 3),
                'direction': direction
            }

            # Print to console
            self.get_logger().info(str(log_entry))

            # Append to in-memory log
            self.data_log.append(log_entry)

            # Write to JSON file every 10 entries
            if len(self.data_log) % 10 == 0:
                with open(self.json_file_path, 'w') as f:
                    json.dump(self.data_log, f, indent=2)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    """
    Main function that initializes and runs the ROS 2 node.
    """
    rclpy.init(args=args)
    node = DataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
