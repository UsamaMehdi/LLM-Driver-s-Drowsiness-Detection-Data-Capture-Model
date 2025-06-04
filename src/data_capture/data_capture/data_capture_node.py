import rclpy
from rclpy.node import Node
import carla
import math
import json

from std_msgs.msg import String, Float32, Float32MultiArray

class DataCaptureNode(Node):
    def __init__(self):
        super().__init__('data_capture_node')

        # Initialize Carla client
        self._client = carla.Client("localhost", 2000)
        self._client.set_timeout(10.0)
        self._world = self._client.get_world()

        # Get the vehicle actor
        try:
            self._vehicle = self._world.get_actors().filter("vehicle.*")[0]
            self.get_logger().info(f'Found vehicle: {self._vehicle.type_id}')
        except IndexError:
            self.get_logger().error('No vehicle found in the world.')
            return

        # Init ROS 2 publishers
        self._legacy_publisher = self.create_publisher(String, '/data_capture/legacy', 10)
        self._position_publisher = self.create_publisher(Float32MultiArray, '/data_capture/vehicle_position', 10)
        self._lane_center_publisher = self.create_publisher(Float32MultiArray, '/data_capture/lane_center', 10)
        self._steering_angle_publisher = self.create_publisher(Float32, '/data_capture/steering_angle', 10)
        self._lateral_offset_publisher = self.create_publisher(Float32, '/data_capture/lateral_offset', 10)
        self._direction_publisher = self.create_publisher(String, '/data_capture/direction', 10)

        # Timer to capture data periodically
        self._timer = self.create_timer(0.5, self.capture_data)

    def _get_signed_lateral_offset(self, vehicle_location):
        map = self._world.get_map()
        waypoint = map.get_waypoint(vehicle_location, project_to_road=True)
        lane_center = waypoint.transform.location
        lane_forward = waypoint.transform.get_forward_vector()

        # Vector from lane center to vehicle
        dx = vehicle_location.x - lane_center.x
        dy = vehicle_location.y - lane_center.y

        # Normalize lane forward vector (2D)
        fx, fy = lane_forward.x, lane_forward.y
        mag_f = math.sqrt(fx**2 + fy**2)
        fx /= mag_f
        fy /= mag_f

        # Cross product to determine direction
        cross = fx * dy - fy * dx

        # Distance (magnitude)
        distance = math.sqrt(dx**2 + dy**2)

        # Signed distance
        signed_offset = distance if cross > 0 else -distance
        return lane_center, signed_offset
    
    def capture_data(self):
        transform = self._vehicle.get_transform()
        location = transform.location
        control = self._vehicle.get_control()
        steering_angle = control.steer

        lane_center, lateral_offset = self._get_signed_lateral_offset(location)

        # Prepare data for publishing
        position_msg = Float32MultiArray(data=[round(location.x, 4), round(location.y, 4), round(location.z, 2)])
        lane_center_msg = Float32MultiArray(data=[round(lane_center.x, 4), round(lane_center.y, 4), round(lane_center.z, 2)])
        steering_angle_msg = Float32(data=round(steering_angle, 4))
        lateral_offset_msg = Float32(data=round(lateral_offset, 4))
        direction_msg = String(data="left" if lateral_offset > 0 else "right" if lateral_offset < 0 else "center")

        # Publish messages
        self._position_publisher.publish(position_msg)
        self._lane_center_publisher.publish(lane_center_msg)
        self._steering_angle_publisher.publish(steering_angle_msg)
        self._lateral_offset_publisher.publish(lateral_offset_msg)
        self._direction_publisher.publish(direction_msg)

        # Legacy message for compatibility
        data = {
            "vehicle_position": {"x": round(location.x, 4), "y": round(location.y, 4), "z": round(location.z, 2)},
            "lane_center": {"x": round(lane_center.x, 4), "y": round(lane_center.y, 4), "z": round(lane_center.z, 2)},
            "steering_angle": round(steering_angle, 4),
            "lateral_offset": round(lateral_offset, 4),
            "direction": "left" if lateral_offset > 0 else "right" if lateral_offset < 0 else "center"
        }
        self._legacy_publisher.publish(String(data=json.dumps(data)))

        # Log and save data
        # self.get_logger().info('Captured data.')


    def _destroy_node_and_cleanup(self):
        self.get_logger().info('Destroying node and cleaning up...')
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    data_capture_node = DataCaptureNode()

    rclpy.spin(data_capture_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    data_capture_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
