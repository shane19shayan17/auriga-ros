import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry


class MapVisualizer(Node):

    def __init__(self):
        super().__init__('map_visualizer')

        self.map_ = None
        self.width_ = 20
        self.height_ = 20

        keep_last_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            keep_last_qos
        )

        self.subscribe_robot_pose = self.create_subscription(
                    Odometry,
                    '/robot_pose',
                    self.robot_pose_callback,
                    10
                )
        
        self.cmap = ListedColormap(['#ffffff', "#691e89", "#d60e9e", "#1325ce"])
        self.norm = BoundaryNorm(
            # free  |wall|table|robot
            #    0  |  1  | 9 | 99      
            [-1.5, 0.5, 1.5, 10, 101],
            self.cmap.N
        )

        # Create the initial map
        grid = np.full(
            (self.height_, self.width_),
            -1,
            dtype=np.int16
        )

        self.fig, self.ax = plt.subplots()

        self.image = self.ax.imshow(
            grid,
            cmap=self.cmap,
            norm=self.norm,
            origin='lower',
            interpolation='nearest'
        )

        self.last_shape = grid.shape
        self.configure_axes(self.width_, self.height_)

        plt.show(block=False)

        self.timer = self.create_timer(
            0.1,
            self.visualize_callback
        )

        self.get_logger().info('Map visualizer node ready!')

    def robot_pose_callback(self, msg: Odometry):
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        
        robot_x = int(np.round(robot_x))
        robot_y = int(np.round(robot_y))
        
        self.map_[robot_x, robot_y] = 99 # simple number for robot

    def map_callback(self, msg: OccupancyGrid):
        # Read map dimensions
        self.width_ = msg.info.width
        self.height_ = msg.info.height

        # Convert flat data into a 2D grid
        grid = np.asarray(msg.data, dtype=np.int16)
        grid = grid.reshape((self.height_, self.width_))

        # Store the latest received map
        self.map_ = grid

    def configure_axes(self, width, height):
        self.ax.set_xlim(-0.5, width - 0.5)
        self.ax.set_ylim(-0.5, height - 0.5)

        self.ax.set_xticks(np.arange(-0.5, width, 1), minor=True)
        self.ax.set_yticks(np.arange(-0.5, height, 1), minor=True)

        self.ax.grid(
            which='minor',
            color='black',
            linewidth=0.5
        )

        self.ax.set_xticks(range(width))
        self.ax.set_yticks(range(height))

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Map')

    def visualize_callback(self):
        # Update existing image
        self.image.set_data(self.map_)

        # Adjust axes if map dimensions change
        self.fig.canvas.draw_idle()

        # # Process Matplotlib GUI events
        plt.pause(0.005)


def main(args=None):
    rclpy.init(args=args)

    node = MapVisualizer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        plt.close('all')
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()