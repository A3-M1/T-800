#!/usr/bin/python3

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
#from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped


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
        #self._sub_map = rosNode.create_subscription(OccupancyGrid, '/map',self.map_callback, 10)
        self._sub_map_metadata = rosNode.create_subscription(MapMetaData, '/map_metadata',self.map_metadata_callback, 10)
        #self._sub_tf= rosNode.create_subscription(TFMessage, '/tf',self.tf_callback, 10)
        self._sub_tf= rosNode.create_subscription(PoseWithCovarianceStamped, '/pose',self.pose_callback, 10)

        # Log the start
        self._logger.info('Started !')


    # Map callback
    #def map_callback(self, msg):
        #self._logger.info(msg.data)

    # Map_metadata callback
    def map_metadata_callback(self, msg):
        self._logger.info(f"Resolution: {msg.resolution}")
        self._logger.info(f"Width: {msg.width}")
        self._logger.info(f"Height: {msg.height}")
      
    # Tf callback
    #def tf_callback(self, msg):
    #   self._logger.info(f"Transform: {msg.transforms}")
    
    # Tf callback
    def pose_callback(self, msg):
        self._logger.info(f"Pose: {msg.pose}")


# Execute the function.
if __name__ == "__main__":
    startNode()
