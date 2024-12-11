from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import numpy as np

IMAGE_WIDTH = 1400

# TODO: Add your new constants here

TIMEOUT = 5.0 #TODO threshold in timer_callback
SEARCH_YAW_VEL = 0.3 #TODO searching constant
TRACK_FORWARD_VEL = 0.3 #TODO tracking constant
KP = 3.0 #TODO proportional gain for tracking

class State(Enum):
    SEARCH = 0
    TRACK = 1

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = State.TRACK

        # TODO: Add your new member variables here
        self.kp = KP # TODO
        self.most_centered = 1
        self.most_recent_detection = 0
        self.current_time = 0

    def detection_callback(self, msg):
        """
        Determine which of the HAILO detections is the most central detected object
        """
        self.current_time = msg.header.stamp.sec
        self.most_centered = 1
        for i in msg.detections:
            x = i.bbox.center.position.x
            y = i.bbox.center.position.y

            x = (x - (IMAGE_WIDTH/2)) / (IMAGE_WIDTH/2)
            
            if abs(x) < abs(self.most_centered):
                    self.most_centered = x 
                    self.most_recent_detection = msg.header.stamp.sec
                    #self.get_logger().info(f'x: {self.most_centered}')
                    #self.get_logger().info(f'time: {self.most_recent_detection}')
                # self.get_logger().info(x)

    def timer_callback(self):
        """
        Implement a timer callback that sets the moves through the state machine based on if the time since the last detection is above a threshold TIMEOUT
        """
        
        if self.current_time - self.most_recent_detection > TIMEOUT: # TODO: Part 3.2
            self.state = State.SEARCH
        else:
            self.state = State.TRACK

        yaw_command = 0.0
        forward_vel_command = 0.0

        if self.state == State.SEARCH:
            yaw_command = SEARCH_YAW_VEL if self.most_centered < 0 else -SEARCH_YAW_VEL
        elif self.state == State.TRACK:
            yaw_command  = self.kp*(-self.most_centered)
            forward_vel_command = TRACK_FORWARD_VEL
        self.get_logger().info(f'yaw: {yaw_command}')
            # pass # TODO: Part 2 / 3.4

        cmd = Twist()
        cmd.angular.z = yaw_command
        cmd.linear.x = forward_vel_command
        self.command_publisher.publish(cmd)

def main():
    rclpy.init()
    state_machine_node = StateMachineNode()

    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        zero_cmd = Twist()
        state_machine_node.command_publisher.publish(zero_cmd)

        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()