#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

#vicon
import pyvicon_datastream as pv
from pyvicon_datastream import tools
VICON_TRACKER_IP = "10.0.108.3"
OBJECT_NAME = "SpeedRacer"

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

        # julia stuff
        self.JuliaMain = self.julia_bringup()

        # vicon stuff
        self.ViconClient = self.vicon_bringup(VICON_TRACKER_IP)
        # un-comment for real ViconTracker in lab
        #self.ViconTracker = tools.ObjectTracker(VICON_TRACKER_IP)

        # vicon mocking (for outside of vicon lab)
        self.MockTick = 0
        self.MockX = 0
        self.MockY = 0
        self.MockZ = 0
        self.MockEulerX = 0
        self.MockEulerY = 0
        self.MockEulerZ = 0

        self.cmd_vel_pub_ = self.create_publisher(Twist,"/cmd_vel",10)

        timer_period = 3.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Move forward node has been started")
 
    def vicon_bringup(self, tracker_ip):
        vicon_client = pv.PyViconDatastream()
        # testing for now
        return vicon_client
        # in real world need to actually connect
        ret = vicon_client.connect(tracker_ip)

        if ret != pv.Result.Success:
            print(f"Connection to {tracker_ip} failed")
            raise Exception("could not connect to vicon.")
        else:
            print(f"Connection to {tracker_ip} successful")
            return vicon_client

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
        #
        # Mock Vicon Test
        #
        # Get current robot state
        inputs = get_state()
        input_x = inputs[2][0][0]
        input_y = inputs[2][0][1]
        input_speed = 0.2 # gas Gas GAS
        input_heading = inputs[2][0][3]
        outputs = self.JuliaMain.new_bearings(input_x, input_y, input_speed, input_heading)
        x_vel = outputs[0]
        y_vel = outputs[1]
        self.send_velocity_command(x_vel)
        # Send current state to julia

        #
        # coin toss test
        #
        #outputs = self.JuliaMain.coin_toss()
        #print("outputs is: ", outputs)
        # Update robot state
        #update_my_state(outputs)
        #if outputs == 1:
        #    self.send_velocity_command(0.2)
        #else:
        #    self.send_velocity_command(0.0)

    def get_state(self, object_name):
        self.MockX = self.MockX + 0.1
        self.MockY = 0
        self.MockZ = 0
        self.MockEulerX = 0
        self.MockEulerY = 0
        self.MockEulerZ = 0
        self.MockTick = self.MockTick + 1
        return 0.010, self.MockTick, [[ "MockSpeedRacer", "MockBody", self.MockX, self.MockY, self.MockZ, self.MockEulerX, self.MockEulerY, self.MockEulerZ ]]
        #position = self.ViconTracker.get_position(object_name)
        #print(f"Position: {position}")
        #
        # position = (
        #   latency,
        #   framenumber,
        #   position = [
        #      [
        #        subject_name,
        #        segment_name,
        #        position_x,
        #        position_y,
        #        position_z,
        #        euler_x,
        #        euler_y,
        #        euler_z
        #      ]
        #    ]
        # )
        #
        #return position

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
