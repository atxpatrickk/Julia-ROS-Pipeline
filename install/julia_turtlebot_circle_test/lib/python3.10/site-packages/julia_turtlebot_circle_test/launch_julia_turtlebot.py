#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def main(args=None):
    #
    # Set up our nodes
    #
    rclpy.init(args=args)
    vel_node = MoveForward()
    rclpy.spin(vel_node)
    rclpy.shutdown()

# not really used at this point.
def julia_bootstrap():
    #
    # ensure that PyJulia has installed PyCall, and has added its necessary
    # Julia packages
    #
    import julia
    julia.install()

class MoveForward(Node):
    def __init__(self):
        super().__init__("launch_julia_turtlebot")
        self.JuliaMain = self.julia_bringup()
        self.cmd_vel_pub_ = self.create_publisher(Twist,"/cmd_vel",10)
        #self.timer = self.create_timer(0.5,self.send_velocity_command)
        self.get_logger().info("Move forward node has been started")
        timer_period = 3.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def julia_bringup(self):
        #
        # Set the path to your Julia script
        #
        #print("before script_path")
        #script_path = "/home/ubuntu/ros2_ws/src/julia_turtlebot_circle_test/julia_turtlebot_circle_test/turtlebot_circle.jl"
        #script_path = "/home/ubuntu/ros2_ws/src/julia_turtlebot_circle_test/julia_turtlebot_circle_test/hello_world.jl"
        script_path = "/home/ubuntu/ros2_ws/src/julia_turtlebot_circle_test/julia_turtlebot_circle_test/sanity_check.jl"
        #print("after script path")

        #
        # import Julia() constructor
        #
        #print("before import julia")
        from julia import Julia
        #print("after import julia")

        #
        # Instantiate an instance of PyJulia with that Julia() constructor.
        # Apparently, PyJulia doesn't play nice with static linking of python to
        # libpython, so you gotta do this "compiled_modules=False" dance, which is
        # slow, in order for PyJulia to actually work. Their attitude is "lol
        # Debian/Ubuntu, who uses *those* distributions?" Seriously?  Whatever.
        #
        print("Julia bringup (takes a while)...")
        jl = Julia(compiled_modules=False)
        print("Julia bringup finished.")

        #
        # Now that we have an instantiation of PyJulia in jl, we can, in theory,
        # import Main, a wrapper for the julia interpreter (is it actually an
        # interpreter? I dunno.)
        #
        #print("before import main")
        from julia import Main
        #print("after import main")

        #
        # Now, finally, we can tell our instantiated PyJulia, via Main(), to pull
        # in our external Julia script and run it.
        #
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
    #
    # bootstrap PyJulia
    #
    #print("before julia_bootstrap")
    jl = julia_bootstrap()
    #print("after julia_bootstrap")

    #
    # execute main
    #
    #print("before main")
    main()
    #print("after main")
