#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState



class Gazebo(Node):

    def __init__(self):
        super().__init__('gazebo')
        self.srv = self.create_service(SetModelState, '/gazebo/set_model_state', self.set_model_state_callback)
        
    def set_model_state_callback(self,req,res):
        self.get_logger().info('Incoming request\n {}'.format(req))
        return res
    
def main(args=None):

    rclpy.init(args=None)
    node = Gazebo()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
 
