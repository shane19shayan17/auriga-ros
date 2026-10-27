import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from interface.msg import Matrix2D, Pose2D
import numpy as np
from enum import Enum

class MapContext(Enum):
    FREE_SPACE = 0,
    WALL = 1,
    TABLE = 9,
    ROBOT = 99,

class MapLoaderNode(Node):

    def __init__(self):
        super().__init__('map_loader')
        
        self.map_path = '/home/shayo/coding/auriga-ros/src/map_loader/map_loader/map.txt'
        self.frame_id_ = 'map'
        self.resolution_ = 1.0
        self.height_ = 20
        self.width_ = 20
        
        qos_setting = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher_ = self.create_publisher(
            OccupancyGrid,
            "/map",
            qos_setting
        )
        
        self.tables_pose_publisher_ = self.create_publisher(
            Matrix2D,
            "tables_pose",
            qos_setting
        )
        
        
        if(self.publishMapOnce()):
            self.get_logger().info(f'map loaded successfuly !')
        else:
            self.get_logger().warn(f'map was not loaded')
        
    def publishMapOnce(self) -> bool:
        map_ = OccupancyGrid()
        map_.header.stamp = self.get_clock().now().to_msg()
        map_.header.frame_id = self.frame_id_
        map_.info.resolution = self.resolution_
        map_.info.height = self.height_
        map_.info.width = self.width_
        map_.info.origin.position.x = 0.0
        map_.info.origin.position.y = 0.0
        map_.info.origin.orientation.w = 1.0 # no rotation
        
        try:
            with open(self.map_path, 'r') as f:
                raw_lines = f.readlines()
        except OSError:
            self.get_logger().error(f"Cannot open '{self.map_path}'")
            return False
        
        rows = []
        for line in raw_lines:
            line = line.rstrip('\r\n ')
            if line:
                rows.append(line)
        
        # means some data missed !
        if len(rows) != self.height_:
            self.get_logger().error(f'Expected {self.height_} rows, got {len(rows)}')
            return False

        data = ''.join(rows)
        data = list(map(int, data)) # convert all string to int

        map_.data = data

        self.map_publisher_.publish(map_)
        
        # publish tables pose
        tables_pose = Matrix2D()
        
        for i in range(len(rows)):
            for j in range(len(rows[i])):
                if int(rows[i][j]) == MapContext.TABLE.value[0]:
                    pose = Pose2D()
                    pose.x = i
                    pose.y = j
                    tables_pose.poses.append(pose)
        
        tables_pose.rows = len(tables_pose.poses)
        self.tables_pose_publisher_.publish(tables_pose)
        
        return True

def main(args=None):
    rclpy.init(args=args)

    robot_node = MapLoaderNode()

    rclpy.spin(robot_node)

    robot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()