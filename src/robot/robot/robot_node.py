import rclpy
from rclpy.node import Node

from interface.msg import Robot
from interface.srv import FoodReady
from nav_msgs.msg import OccupancyGrid

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import time

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
        
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscriber_ = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            map_qos
        )
        self.grid_ = []

    def timer_callback(self):
        msg = Robot()
        self.robot_state_ = self.handle_robot_state()
        msg.robot_state = self.robot_state_
        self.publisher_.publish(msg)
    
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
        
        for y in range(height):
            for x in range(width):
                self.grid_[y][x] = grid_map.data[y * width + x]
        self.get_logger().info(
            f'Got map {width}x{height}, resolution {grid_map.info.resolution:.2f} m/cell')
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