import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

from grid_nav.map_utils import parse_text_map, occupancy_data


class MapPublisher(Node):
    """Reads a text map and publishes it ONCE on /map with transient_local QoS.

    transient_local = "latched": late subscribers (the Nav2 costmap starts
    after us, or before us) still receive the last message. With the default
    volatile QoS the costmap would never see the map.
    """

    def __init__(self):
        super().__init__('map_publisher')
        self.declare_parameter('map_file', '')
        self.declare_parameter('resolution', 1.0)
        self.declare_parameter('frame_id', 'map')

        path = self.get_parameter('map_file').value
        res = float(self.get_parameter('resolution').value)
        frame = self.get_parameter('frame_id').value
        if not path:
            raise RuntimeError("parameter 'map_file' is required")

        with open(path, 'r', encoding='utf-8') as f:
            gm = parse_text_map(f.read(), resolution=res)

        msg = OccupancyGrid()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = res
        msg.info.width = gm.width
        msg.info.height = gm.height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = occupancy_data(gm)

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, 'map', qos)
        self.pub.publish(msg)
        self.get_logger().info(
            f'published {gm.width}x{gm.height} map, start={gm.start} goal={gm.goal}')


def main():
    rclpy.init()
    node = MapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
