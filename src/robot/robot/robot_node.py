import rclpy
from rclpy.node import Node

from interface.msg import Robot
from interface.srv import FoodReady
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.impl.rcutils_logger import RcutilsLogger

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import time

class Robothandler():
    def __init__(self, logger: RcutilsLogger):
        self.kitchen_odom = Odometry()
        
        # initial state of robot
        self.robot_odom = Odometry()
        self.robot_odom.pose.pose.position.x = 2.0
        self.robot_odom.pose.pose.position.y = 2.0
        self.robot_odom.pose.pose.orientation.w = 1.0
        
        self.logger = logger
    
    def set_kitchen_odom(self, position : Odometry):
        self.kitchen_odom = position
    
    def get_robot_odom(self) -> Odometry:
        return self.robot_odom

class RobotNode(Node):

    def __init__(self):
        super().__init__('robot_node')
        self.publisher_ = self.create_publisher(
                                Robot,
                                'robot_state',
                                10)
        self.timer_period = 0.20 # second
        self.start_time_ = time.time() # second
        
        self.timer = self.create_timer(
                        self.timer_period,
                        self.timer_callback)
        
        self.robot_state_ = "free"
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
        
        self.grid_ = []
        self.robotHandler = Robothandler(self.get_logger())
        
        self.get_logger().info(f'Robot node started !')

    def timer_callback(self):
        msg = Robot()
        self.robot_state_ = self.handle_robot_state()
        msg.robot_state = self.robot_state_
        self.publisher_.publish(msg)
        
        self.odom_ = self.robotHandler.get_robot_odom()
        self.odometry_publisher_.publish(self.odom_)
    
    def kitchen_position_callback(self, msg : Odometry):
        self.robotHandler.set_kitchen_odom(msg)
    
    def handle_robot_state(self) -> str:
        elapsed_time = time.time() - self.start_time_ 
        
        state = "free"
        if(elapsed_time > 2 and elapsed_time < 4):
            state = "busy"
        
        # reset timer
        if(elapsed_time > 5):
            self.start_time_ = time.time()
        
        return state

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
        for row in grid:
            line = ''
            for cell in row:
                if cell > 0:
                    line += '#'
                elif cell == 0:
                    line += '.'
                else:
                    line += '?'
            self.get_logger().info(line)

def main(args=None):
    rclpy.init(args=args)

    robot_node = RobotNode()

    rclpy.spin(robot_node)

    robot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()