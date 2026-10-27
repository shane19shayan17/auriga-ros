import rclpy
from rclpy.node import Node

from interface.msg import Robot
from interface.srv import FoodReady, Order
from nav_msgs.msg import OccupancyGrid, Odometry, Path

# rclpy types
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.task import Future

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from enum import Enum
from collections import deque
from dataclasses import dataclass

class State(Enum):
    FREE = 1,
    COMMING_BACK = 2,
    DELIVERING = 3,
    ORDERING = 4,
    NAVIGATING = 5,


@dataclass
class Duty:
    duty_type: State = State.FREE
    succeed: bool = False
    table_pose: tuple = (0, 0)


class Robothandler():
    def __init__(self, logger: RcutilsLogger):
        self.kitchen_odom = Odometry()
        
        # initial state of robot
        self.robot_odom = Odometry()
        self.robot_odom.pose.pose.position.x = 2.0
        self.robot_odom.pose.pose.position.y = 2.0
        self.robot_odom.pose.pose.orientation.w = 1.0
        
        self.logger = logger
        
        self.robot_state: State = State.COMMING_BACK
        
        self.robot_path_: list[tuple] = []
        self.duties_: deque[Duty] = deque()
    
    def set_kitchen_odom(self, position : Odometry):
        self.kitchen_odom = position
    
    def get_robot_odom(self) -> Odometry:
        return self.robot_odom
    
    def set_robot_state(self, state: State) -> None:
        self.robot_state = state
    
    def get_robot_state(self) -> int:
        return self.robot_state.value[0]
    
    def get_robot_state(self) -> State:
        return self.robot_state
    
    def set_robot_path(self, path: list[tuple]):
        self.robot_path_ = path
    
    def navigate(self):
        # check wheater robot is navigating 
        if self.robot_state != State.COMMING_BACK \
            and self.robot_state != State.DELIVERING:             
                return
        
        # check the path is generated
        if self.robot_path_ == []:
            self.logger.debug(f'path was not generated !')
            return
        
        self.robot_odom.pose.pose.position.x = self.robot_path_[0][0]
        self.robot_odom.pose.pose.position.y = self.robot_path_[0][1]
        self.logger.info(f'robot moved to ({self.robot_path_[0][0]}, {self.robot_path_[0][1]})')
        self.robot_path_.pop(0)
    
    def handle_new_order(self, order: Order.Request) -> bool:
        try:
            duty = Duty(State.ORDERING, False, (order.table_pose[0], order.table_pose[1]))
            self.duties_.append(duty)
            return True
        except Exception as e:
            self.logger.warn(f'order did not accept !\n{e}')
            return False
    
    def handle_food_received(self, ):
        pass
    
    # main function of robot. this function call every loop of ros timer
    def run(self) -> None:
        # check if duty is empty
        if not self.duties_:
            self.logger.info('robot is free and has no duty')
            self.robot_state = State.FREE
            return
        
        duty = self.duties_[0]
        if duty.duty_type == State.ORDERING:
            # compute path to pose
            goal_pose = duty.table_pose
            self.logger.info(f'goal ->({goal_pose[0]}, {goal_pose[1]})')
            # change state to navigating
        elif duty.duty_type == State.NAVIGATING:
            # control robot to acheive to goal
            pass


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
        
        # receive path that navFn generated
        self.path_subscriber_ = self.create_subscription(
            Path,
            "/plan",
            self.path_callback,
            10
        )
        
        self.grid_ = []
        self.robotHandler = Robothandler(self.get_logger())
        
        # order service to give order from customer
        self.order_service_ = self.create_service(
            Order,
            "order",
            callback=self.order_callback
        )
        
        # client for send order to kitchen
        self.send_order_client_ = self.create_client(
            Order,
            "customer_order"
        )
        
        self.get_logger().info(f'Robot node started !')

    def timer_callback(self):
        self.robotHandler.run()
        
        msg = Robot()
        msg.robot_state = self.robotHandler.get_robot_state().value[0]
        self.robot_state_publisher.publish(msg)
        
        self.robotHandler.navigate()
        
        odom = self.robotHandler.get_robot_odom()
        self.odometry_publisher_.publish(odom)
    
    def kitchen_position_callback(self, msg : Odometry):
        self.robotHandler.set_kitchen_odom(msg)

    def food_recieve_callback(self, request, response):
        if self.robotHandler.get_robot_state() == State.FREE:
            pass
        self.get_logger().info(f"order {request.name} was delivered !")
        return response
    
    def map_callback(self, grid_map : OccupancyGrid):
        width = grid_map.info.width
        height = grid_map.info.height
        
        grid = [[-1] * width for _ in range(height)]
        
        for y in range(height):
            for x in range(width):
                grid[y][x] = grid_map.data[y * width + x]
        self.print_grid(grid)
    
    def print_grid(self, grid : list):
        self.get_logger().debug(f'{grid}')
    
    def path_callback(self, msg: Path):
        path_lenght = len(msg.poses)
        
        # check wheaher the line is empty
        if path_lenght < 0:
            return
        
        path: list[tuple] = []
        for pose in msg.poses:
            path.append((pose.pose.position.x, pose.pose.position.y))
        self.robotHandler.set_robot_path(path)
    
    def order_callback(self, request: Order.Request, response: Order.Response):
        response.success = self.robotHandler.handle_new_order(request)
        return response
    
    def send_order_to_kitchen(self, order: Order.Request):
        while not self.send_order_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the kitchen server ...')
        
        future: Future = self.send_order_client_.call_async(order)
        future.add_done_callback(self.send_food_done_callback)
        
    
    def send_food_done_callback(self, future: Future):
        response = future.result()
        pass


def main(args=None):
    rclpy.init(args=args)

    robot_node = RobotNode()

    rclpy.spin(robot_node)

    robot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()