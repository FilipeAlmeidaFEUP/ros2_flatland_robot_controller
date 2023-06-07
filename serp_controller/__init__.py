#!/usr/bin/env python3
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from flatland_msgs.srv import MoveModel
from flatland_msgs.msg import Collisions

import math

# To change the map go to the file world/layer.yaml
# There you can choose one of three maps
# You can also add your own map in an image to the world folder and add it to the layer file
# To understand the layer file configuration, consult https://flatland-simulator.readthedocs.io/en/latest/core_functions/layers.html

class SerpController(Node):
    def __init__(self) -> None:
        super().__init__("SerpController")
        # **** Create publishers ****
        self.pub:Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        # ***************************

        # **** Create subscriptions ****
        self.create_subscription(LaserScan, "/static_laser", self.processLiDAR, 1)

        self.create_subscription(Collisions, "/collisions", self.processCollisions, 1)
        # ******************************

    # Change the speed of the robot by publishing a Twist message
    # For more information on the data contained in a Twist message, consult http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html
    def change_robot_speeds(self, publisher, linear, angular):
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        publisher.publish(twist_msg)

    # Send a request to move a model by sending the MoveModel.Request mesasge to the Flatland MoveModel service
    # For more information on the data contained in a MoveModel.Request message, consult https://flatland-simulator.readthedocs.io/en/latest/core_functions/ros_services.html#moving-models
    def move_model(self, model_name, x, y, theta):
        client = self.create_client(MoveModel, "/move_model")
        client.wait_for_service()
        request = MoveModel.Request()
        request.name = model_name
        request.pose = Pose2D()
        request.pose.x = x
        request.pose.y = y
        request.pose.theta = theta
        client.call_async(request)
    
    # Handle LiDAR data
    def processLiDAR(self, data):
        #########################################
        #
        # Write your code here
        #
        # The values from the LiDAR are in data.ranges
        # For more information on the data contained in a LaserScan message, consult http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
        #
        ########################################

        ########################################
        ## Example 1: A robot that moves forward
        # A positive linear velocity makes the robot move forward
        # Zero angular velocitiy prevents the robot from turning/rotating

        #self.change_robot_speeds(self.pub, 1.0, 0.0)
        ########################################

        ########################################
        ## Example 2: A robot that moves in a circle
        # A positive lenear velocity makes the robot move forward
        # A negative angular velocitiy makes the robot turn clockwise

        #self.change_robot_speeds(self.pub, 0.05, -1.0)
        ########################################

        ########################################
        ## Example 3: A robot that moves in circles, and turns around itself when it finds an obstacle in front of it
        ## Find if the laser in front of the robot is detecting an obstacle too close to them

        #num_lasers = len(data.ranges)
        #max_distance = 1.0 # maximum allowed distance for an obstacle in one of three sensor readings before the robot starts spining around itself
        #middle_laser_range = data.ranges[math.floor(num_lasers / 2 + 0.5)]
        ## if the distance reading is invalid or the distance is below the maximum allowed distance
        #if( not (middle_laser_range >= data.range_min and middle_laser_range <= data.range_max) or middle_laser_range < max_distance): 
        #   # ROTATE
        #    self.change_robot_speeds(self.pub, 0.0, -2.0)
        #else:
        #   # MOVE FORWARD
        #    self.change_robot_speeds(self.pub, 1.0, -2.0)
        ########################################

        return
    
    # Process collisions
    def processCollisions(self, data):
        #########################################
        #
        # Write your code here to handle collisions here
        #
        # There is a list of all collisions that ocurred in data.collisions
        # For more information on the data contained in a Collisions and Collision message, consult https://flatland-simulator.readthedocs.io/en/latest/included_plugins/bumper.html
        #
        ########################################

        ########################################
        ## Example: Send the robot back to origin if there is a collision
    
        #if len(data.collisions) > 0:
        #    self.move_model('serp', 0.0, 0.0, -1.57079632679)
        ########################################

        return

def main(args = None):
    rclpy.init()
    
    serp = SerpController()

    rclpy.spin(serp)

if __name__ == "__main__":
    main()
