#!/usr/bin/python3

import numpy as np
import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import random as rd

# Start the node
def startNode():
    rclpy.init()
    aNode= Node("slam_map")
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
        self._sub_map = rosNode.create_subscription(OccupancyGrid, '/map',self.map_callback, 10)
        # Log the start
        self._logger.info('Started !')

    # Map callback
    def map_callback(self, msg):
        # 100 = mur
        # -1 = jsp
        # 0 = libre
        # self._logger.info(msg.data)
        slam_map = np.reshape(msg.data, (msg.info.width, msg.info.height ))
        # self._logger.info(str(slam_map))
        # self.point_aleatoire(msg)

    #def point_aleatoire(self, msg):
    #    slam_map = np.reshape(msg.data, (msg.info.width, msg.info.height ))
    #    tab_points = [] # chart for the points unexplored by the robot
    #    # addition of the points unexplored
    #    for x in range (len(slam_map)):
    #        for y in range (len(slam_map[x])):
    #            if slam_map[x][y] == -1 :
    #                tab_points.append((x,y))
    #    # random selection of a point unexplored
    #    point = tab_points[rd.randint(0, len(tab_points)-1)]

    #    self._logger.info(str(point))

    def gros_pixel(self,msg):
        somme_gros_pixel = 0
        slam_map = np.reshape(msg.data, (msg.info.width, msg.info.height ))
        for i in range (len(slam_map) - 10):
            for j in range (len(slam_map) - 10) :
                gros_pixel = slam_map[i : i+10][j : j+10]
                if np.sum(gros_pixel) < 0 and np.sum(gros_pixel) > -100 and np.sum(gros_pixel) < somme_gros_pixel :
                    somme_gros_pixel = np.sum(gros_pixel)  
                    best_i = i + 5
                    best_j = j + 5
    return(best_i, best_j)  

# Execute the function.
if __name__ == "__main__":
    startNode()