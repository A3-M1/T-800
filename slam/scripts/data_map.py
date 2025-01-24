#!/usr/bin/python3

# position par rapport à /map 
# il va vers la position en contournant l'obstacle 
# 
import math
import numpy as np
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Pose, Transform, Vector3, Point, Quaternion

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
        #self._sub_map_metadata = rosNode.create_subscription(MapMetaData, '/map_metadata',self.map_metadata_callback, 10)
        #self._sub_tf= rosNode.create_subscription(TFMessage, '/tf',self.tf_callback, 10)
        #self._sub_tf= rosNode.create_subscription(PoseWithCovarianceStamped, '/pose',self.pose_callback, 10)
        #self._angle = 0
        #self._distance = 0
        # Log the start
        self._logger.info('Started !')

    # Map callback
    def map_callback(self, msg):
        # 100 = mur
        # -1 = jsp
        # 0 = libre
        self._logger.info(msg.data)
        #tab_map = np.reshape(msg.data, (msg.info.width, msg.info.height )
        #self._logger.info(str(tab_map)))


    # Map_metadata callback
    #def map_metadata_callback(self, msg):
    #    self._logger.info(f"Resolution: {msg.resolution}")
    #    self._logger.info(f"Width: {msg.width}")
    #    self._logger.info(f"Height: {msg.height}")

    # Tf callback
    #def tf_callback(self, msg):
    #    if len(msg.transforms) > 0: 
    #        transform = msg.transforms[0]  
     #       translation = transform.transform.translation  
     #       #self._logger.info(f"Transform translation: x={translation.x}, y={translation.y}, z={translation.z}")
     #       rotation = transform.transform.rotation 
     #       #self._logger.info(f"Transform rotation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")
     #       self._angle = math.atan2(translation.y, translation.x)
     #       self._distance = math.sqrt(translation.x**2 + translation.y**2)

        #if self._angle != -2.163802305531382 and self._distance != 0.024475186458227053:
        #self._logger.info(f"angle={self._angle*57,2958}, distance={self._distance}")
        #self._logger.info(f"Transform: {msg.transforms}")

    # Pose callback
    #def pose_callback(self, msg):
    #    # Access the position
    #    position = msg.pose.pose.position
    #    self._logger.info(f"Position: x={position.x}, y={position.y}, z={position.z}")
    #    orientation = msg.pose.pose.orientation
     #   self._logger.info(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

# Execute the function.
if __name__ == "__main__":
    startNode()
