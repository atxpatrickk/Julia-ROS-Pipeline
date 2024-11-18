using ROS2

# Initialize ROS2 node
node = ROS2Node("turtlebot_circle_node")

# Define the publisher to the cmd_vel topic
pub = ROS2.Publisher(node, "/cmd_vel", "geometry_msgs/msg/Twist")

# Define the message type
msg = ROS2.Message("geometry_msgs/msg/Twist")

# Set linear and angular velocities for circular motion
linear_velocity = 0.2  # Linear velocity in m/s
angular_velocity = 0.5  # Angular velocity in rad/s

# Create a rate object to control the loop frequency
rate = ROS2.Rate(10)  # 10 Hz

try
    while true
        # Set linear and angular velocities
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity

        # Publish the message
        publish(pub, msg)

        # Sleep for the duration of the rate
        sleep(rate)
    end
catch e
    println("An error occurred: $e")
finally
    # Cleanup
    close(pub)
    destroy_node(node)
end