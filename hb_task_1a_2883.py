########################################################################################################################
########################################## eYRC 23-24 Hologlyph Bots Task 1A ###########################################
# Team ID: eYRC#HB#2883
# Team Leader Name: Amritanshu Shandilya
# Team Members Name: Anurag Kumar Singh, Ansh Jaiswal, Saumitra Matta
# College: Presidency College
########################################################################################################################


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math

class snowman(Node):
    def __init__(self):
        '''
        Purpose:
        ---
        Initialise the snowman class by creating the publisher and subscriber
        for the turtlesim and initialise required variable for rest of functionality
        
        Input Arguments:
        ---
            None 

        Returns:
        ---
            None
        '''
        super().__init__('snowman_publisher')
        self.turtle_publish = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pos_sub = self.create_subscription(Pose, '/turtle1/pose', self.position_listener, 10)
        self.cli = self.create_client(Spawn, 'spawn')
        while not self.cli.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        self.second_spawned = False
        self.iswaiting = True

        self.turtle_pos = None
        self.turtle_start_pos = None

        self.circle1_operation_count = 0
        self.circle1_operation_count_max = 3

        self.circle2_operation_count = 0
        self.circle2_operation_count_max = 6

        self.timer = self.create_timer(1.0, self.publish_movement)

    def draw_circle1(self):
        '''
        Purpose:
        ---
        Draw the first small circle keep incrementing the operation count for first circle
        
        Input Arguments:
        ---
            None 

        Returns:
        ---
            None
        '''
        move_cmd = Twist()
        move_cmd.linear.x = 2.0
        move_cmd.angular.z = 2.0
        self.turtle_publish.publish(move_cmd)
        self.get_logger().info('Pos: "%s"' % (self.turtle_pos))
        self.get_logger().info('Task 1: "%s" (Count: %d)\n' % (move_cmd, self.circle1_operation_count + 1))
        self.circle1_operation_count += 1


    def draw_circle2(self):
        '''
        Purpose:
        ---
        Wait for first turtle to stop by checking its linear_velocity. It is done by just exitin 
            the funtion if its greater than 0.1. If the wait is over, set iswaiting to False
        Draw the second bigger circle keep incrementing the operation count for first circle.
        
        Input Arguments:
        ---
            None 

        Returns:
        ---
            None
        '''
        if self.turtle_pos.linear_velocity > 0.1 and self.iswaiting:
            return
        self.iswaiting = False

        move_cmd = Twist()
        move_cmd.linear.x = 2.0
        move_cmd.angular.z = -1.0
        self.turtle_publish.publish(move_cmd)
        self.get_logger().info('Pos: "%s"' % (self.turtle_pos))
        self.get_logger().info('Task 2: "%s" (Count: %d)\n' % (move_cmd, self.circle2_operation_count + 1))
        self.circle2_operation_count += 1

    def spawn_turtle(self):
        '''
        Purpose:
        ---
        Spawn a new turtle at the exact position where the first turtle was Spawned
        update the publisher and subscriber variable to use new turtle instead of last one
        
        Input Arguments:
        ---
            None 

        Returns:
        ---
            None
        '''
        req = Spawn.Request()
        req.x = self.turtle_start_pos.x
        req.y = self.turtle_start_pos.y
        self.future = self.cli.call_async(req)
        self.turtle_publish = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.pos_sub = self.create_subscription(Pose, '/turtle2/pose', self.position_listener, 10)
        self.second_spawned = True
        self.get_logger().info('Spawned a new turtle')


    def publish_movement(self):
        '''
        Purpose:
        ---
        This is the funtion callback for the timer created in constructor. It is responsible
        for orchestrating everything, first wait till you get the starting position of trutle
        , second draw the first circle, then spawn new turtle then draw second circle finally
        stop the timer
        
        Input Arguments:
        ---
            None 

        Returns:
        ---
            None
        '''
        if self.turtle_start_pos is None:
            return

        if self.circle1_operation_count < self.circle1_operation_count_max:
            self.draw_circle1()
        elif not self.second_spawned:
            self.spawn_turtle()
        elif self.circle2_operation_count < self.circle2_operation_count_max:
            self.draw_circle2()
        else:
            self.get_logger().info("DONE, Stopping!!\n")
            self.timer.cancel()

    def position_listener(self, msg):
        '''
        Purpose:
        ---
        This is callback for the position subscriber, this takes the turtle's position.
        This is also responsible to initialise the turtle_start_pos
        
        Input Arguments:
        ---
        `msg` : [turtlesim.msg.Pose]
            Position received on turtlen/pose topic

        Returns:
        ---
            None
        '''
        self.turtle_pos = msg
        if self.turtle_start_pos is None:
            self.turtle_start_pos = self.turtle_pos


def main(args=None):
    rclpy.init(args=args)
    snowman_publisher = snowman()
    try:
        rclpy.spin(snowman_publisher)
    except KeyboardInterrupt:
        pass
    snowman_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
