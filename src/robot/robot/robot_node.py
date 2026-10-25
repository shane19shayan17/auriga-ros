import rclpy
from rclpy.node import Node

from interface.msg import Robot
from interface.srv import FoodReady
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.impl.rcutils_logger import RcutilsLogger

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from enum import Enum

class State(Enum):
    FREE = 1,
    COMMING_BACK = 2,
    DELIVERING = 3

class Robothandler():
    def __init__(self, logger: RcutilsLogger):
        self.kitchen_odom = Odometry()
        
        # initial state of robot
        self.robot_odom = Odometry()
        self.robot_odom.pose.pose.position.x = 2.0
        self.robot_odom.pose.pose.position.y = 2.0
        self.robot_odom.pose.pose.orientation.w = 1.0
        
        self.logger = logger
        
        self.robot_state: State = State.FREE
        
        self.robot_path_: list[tuple] = []
    
    def set_kitchen_odom(self, position : Odometry):
        self.kitchen_odom = position
    
    def get_robot_odom(self) -> Odometry:
        return self.robot_odom
    
    def handle_robot_state(self) -> int:
        if self.robot_odom.pose.pose.position.x == self.kitchen_odom.pose.pose.position.x and self.robot_odom.pose.pose.position.y == self.kitchen_odom.pose.pose.position.y :
            self.robot_state = State.FREE
        else :
            self.robot_state = State.COMMING_BACK
        return self.robot_state.value[0]
    
    def set_robot_path(self, path: list[tuple]):
        self.robot_path_ = path

class RobotNode(Node):

    def __init__(self):
        super().__init__('robot_node')
        self.robot_state_publisher = self.create_publisher(
                                Robot,
                                'robot_state',
                                10)
        self.timer_period = 0.20 # second
        self.timer = self.create_timer(
                        self.timer_period,
                        self.timer_callback)
        
        self.food_recieve_service_ = self.create_service(
            FoodReady,
            "food_ready",
            self.food_recieve_callback
        )
        
        keep_last_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscriber_ = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            keep_last_qos
        )
        
        self.odom_ = Odometry()
        self.odometry_publisher_ = self.create_publisher(
            Odometry,
            "/robot_pose",
            10,
        )
        
        self.kitchen_position_subscriber_ = self.create_subscription(
            Odometry,
            "/kitchen_position",
            self.kitchen_position_callback,
            keep_last_qos,
        )
        
        # recieve path that navFn generated
        self.path_subscriber_ = self.create_subscription(
            Path,
            "/plan",
            self.path_callback,
            10
        )
        
        self.grid_ = []
        self.robotHandler = Robothandler(self.get_logger())
        
        self.get_logger().info(f'Robot node started !')

    def timer_callback(self):
        msg = Robot()
        msg.robot_state = self.robotHandler.handle_robot_state()
        self.robot_state_publisher.publish(msg)
        
        self.odom_ = self.robotHandler.get_robot_odom()
        self.odometry_publisher_.publish(self.odom_)
    
    def kitchen_position_callback(self, msg : Odometry):
        self.robotHandler.set_kitchen_odom(msg)

    def food_recieve_callback(self, request, response):
        self.get_logger().info(f"order {request.name} was delivered !")
        return response
    
    def map_callback(self, grid_map : OccupancyGrid):
        width = grid_map.info.width
        height = grid_map.info.height
        
        self.grid_ = [[-1] * width for _ in range(height)]
        
        for y in range(height):
            for x in range(width):
                self.grid_[y][x] = grid_map.data[y * width + x]
        self.print_grid(self.grid_)
    
    def print_grid(self, grid : list):
        self.get_logger().info(f'{grid}')

    def path_callback(self, msg: Path):
        path_lenght = len(msg.poses)
        
        # check wheaher the line is empty
        if path_lenght < 0:
            return
        
        path: list[tuple] = []
        for pose in msg.poses:
            path.append((pose.pose.position.x, pose.pose.position.y))
        self.robotHandler.set_robot_path(path)

def main(args=None):
    rclpy.init(args=args)

    robot_node = RobotNode()

    rclpy.spin(robot_node)

    robot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()