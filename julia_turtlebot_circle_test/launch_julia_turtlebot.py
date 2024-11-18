#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from pathlib import Path

def main(args=None):
    # Set up our nodes
    
    rclpy.init(args=args)
    vel_node = MoveForward()
    rclpy.spin(vel_node)
    rclpy.shutdown()

def julia_bootstrap():
    
    # ensure that PyJulia has installed PyCall, and has added the necessary Julia packages
    
    import julia
    julia.install()

class MoveForward(Node):
    def __init__(self):
        super().__init__("launch_julia_turtlebot")
        self.JuliaMain = self.julia_bringup()
        self.cmd_vel_pub_ = self.create_publisher(Twist,"/cmd_vel",10)
        self.get_logger().info("Move forward node has been started")
        timer_period = 3.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def julia_bringup(self):
        
        # Set the path to your Julia script
        script_path = str(Path.home()) + "/ros2_ws/src/julia_turtlebot_circle_test/julia_turtlebot_circle_test/sanity_check.jl"

        
        #import Julia() constructor
        
        from julia import Julia

        
        # Instantiate an instance of PyJulia.
        # Must use "compiled_modules=False" in order for PyJulia to actually work (for some reason).
        
        print("Julia bringup (takes a while)...")
        jl = Julia(compiled_modules=False)
        print("Julia bringup finished.")

        
        # With the instantiation of PyJulia in jl, we can import Main: 
        # a wrapper for the julia interpreter
        
        from julia import Main

        
        # Now, we can tell our instantiated PyJulia, via Main(), to pull
        # in our external Julia script and run it.
        
        print("Julia Main includes started.")
        Main.include(script_path)
        print("Julia Main includes finished.")

        return Main

    def timer_callback(self):
        # Get current robot state
        #inputs = get_my_state()

        # Send current state to julia
        outputs = self.JuliaMain.coin_toss()
        print("outputs is: ", outputs)

        # Update robot state
        #update_my_state(outputs)

        if outputs == 1:
            self.send_velocity_command(0.2)
        else:
            self.send_velocity_command(0.0)

    def send_velocity_command(self, velocity=0.0):
        msg = Twist()
        msg.linear.x = velocity
        self.cmd_vel_pub_.publish(msg)
        log_string = "New velocity is: " + str(velocity);
        self.get_logger().info(log_string)

if __name__ == "__main__":

    # bootstrap PyJulia
    jl = julia_bootstrap()

    # execute main
    main()
