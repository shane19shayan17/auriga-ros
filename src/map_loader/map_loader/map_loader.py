import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from interface.msg import Matrix2D, Pose2D


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
        
        self.table_poses: list[tuple] = [
            (2, 2),
            (0, 3),
            (9, 2),
            (7, 3),
            (11, 9),
            (0, 9),
            (0, 7),
            (0, 5),
            (0, 11),
            (0, 13),
            (0, 15),
            (0, 17),
        ]
        
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
        
        data = []
        for line in raw_lines:
            line = line.rstrip('\r\n ')
            str_num: str = ''
            for word in line:
                if word != ' ':
                    str_num += word
                else :
                    data.append(int(str_num))
                    str_num = ''
            data.append(int(str_num))
        map_.data = data

        self.map_publisher_.publish(map_)
        
        # publish tables pose
        tables_pose = Matrix2D()
        
        for pose in self.table_poses:
            p = Pose2D()
            p.x = pose[0]
            p.y = pose[1]
            tables_pose.poses.append(p)
        
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