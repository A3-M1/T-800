#!/usr/bin/python3

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from nav_msgs.msg import OccupancyGrid

# Start the node
def startNode():
    rclpy.init()
    aNode= Node("save_map")
    map = ROSmap(aNode)
    # Start infinite loop
    rclpy.spin(aNode)
    # Clean everything and switch the light off
    aNode.destroy_node()
    rclpy.shutdown()

# Node execution
class ROSmap():
    def __init__(self, rosNode):
        # Logger
        self._logger = rosNode.get_logger()

        # Subscribers to events that can trigger the emergency stop
        self._sub_map = rosNode.create_subscription(OccupancyGrid, '/map',self.map_callback, 10)

        # Log the start
        self._logger.info('Started !')
    
    
    # Button callback
    def button_callback(self, msg):

        self._logger.info('newmap')

# Execute the function.
if __name__ == "__main__":
    startNode()
