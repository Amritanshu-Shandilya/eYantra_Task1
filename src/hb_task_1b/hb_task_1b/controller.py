import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal

DEBUG = False

class HBTask1BController(Node):

    def __init__(self):
        '''
        Purpose:
        ---
        Constructor for HBTask1BController, this is responsible to first create the publisher and
        subscriber for the bot for movement and position. Then it has some variable to store the
        position of the bot, and Kp value which is used in the proportional controller to ensure that the bot reaches the desired location. Finally a service client is made which fetches goal
        for the bot to move. It also waits till the service is available.

        Input Arguments:
        ---
            None

        Returns:
        ---
            None
        '''
        super().__init__('hb_task1b_controller')
        
        # the publisher and subscriber
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.position_listener, 10)

        # Variable to hold bot's current position
        self.hb_x = 0.
        self.hb_y = 0.
        self.hb_theta = 0.

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        # Value for the P-controller
        self.Kp = 1


        # client for the "next_goal" service and wait for service to be created
        self.cli = self.create_client(NextGoal, 'next_goal')      
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = NextGoal.Request() 
        self.index = 0

    def send_request(self, idx):
        '''
        Purpose:
        ---
        This function is response to call the service with the index specified.

        Input Arguments:
        ---
            `idx`: [int]
                Index for service

        Returns:
        ---
            None
        '''
        self.req = NextGoal.Request() 
        self.req.request_goal = idx
        self.future = self.cli.call_async(self.req)

    def log_position(self):
        '''
        Purpose:
        ---
        Just a temporary funtion to print the position of the bot

        Input Arguments:
        ---
            None

        Returns:
        ---
            None
        '''
        self.get_logger().info(f'Pos: ({self.hb_x},{self.hb_y}), \t Theeta: {self.hb_theta}')


    def position_listener(self, msg):
        '''
        Purpose:
        ---
        This funtion is callback for subscriber for /odom. It fetches Odometry position of the bot
        then extract x, y position of bot along with calculate the z-angle (theta) from the quaternion
        using the euler_from_quaternion funtion. Store the values in the respective variable

        Input Arguments:
        ---
            None

        Returns:
        ---
            None
        '''
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.hb_theta = euler_from_quaternion(quaternion=[ori.x, ori.y, ori.z, ori.w])[2]
        self.hb_x = pos.x
        self.hb_y = pos.y


def main(args=None):
    '''
    Purpose:
    ---
    This is main driver for bot, crates an object of controller node, sends request
    to the service to fetch the next goal, with goal and bot's current position 
    claculate the movement velocities for the bot and finally publish it.

    Keep checking if you reach the goal, if you reach increment the index

    Input Arguments:
    ---
        None

    Returns:
    ---
        None
    '''
    rclpy.init(args=args)
    
    ebot_controller = HBTask1BController()
   
    # Send an initial request with the index from ebot_controller.index
    ebot_controller.send_request(ebot_controller.index)
    
    # Main loop
    while rclpy.ok():

        # Check if the service call is done
        if ebot_controller.future.done():
            try:
                # response from the service call
                response = ebot_controller.future.result()
            except Exception as e:
                ebot_controller.get_logger().infselfo(
                    'Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                x_goal      = response.x_goal
                y_goal      = response.y_goal
                theta_goal  = response.theta_goal
                ebot_controller.flag = response.end_of_list
                ####################################################
                
                # Demo for square example
                # x_goal      = [4, -4, -4, 4, 0][ebot_controller.index]
                # y_goal      = [4, 4, -4, -4, 0][ebot_controller.index]
                # theta_goal  = [0, math.pi/2, -math.pi, -math.pi/2, 0][ebot_controller.index]
                # ebot_controller.flag = ebot_controller.index == 4
                
                # Find error (in x, y and theta) in global frame
                error_x = x_goal - ebot_controller.hb_x
                error_y = y_goal - ebot_controller.hb_y
                error_theta = theta_goal - ebot_controller.hb_theta


                bot_real_theeta = -ebot_controller.hb_theta         # the theeta in Odometry is inverse in gobal scope

                # Finally use the error and orientation of the bot and calculate the x and y velocity
                # using coordinate transformation and apply the k controller rate
                v_x = ebot_controller.Kp * (error_x * math.cos(bot_real_theeta) - error_y * math.sin(bot_real_theeta))
                v_y = ebot_controller.Kp * (error_x * math.sin(bot_real_theeta) + error_y * math.cos(bot_real_theeta))

                # extra 0.5 k controller rate for faster angular movement
                w = (ebot_controller.Kp + 0.5) * error_theta        


                # print the values, for debugging purpose
                if DEBUG:
                    print("Bot Pos:", ebot_controller.hb_x, ebot_controller.hb_y, ebot_controller.hb_theta)
                    print("Goal:", x_goal, y_goal, theta_goal, ebot_controller.flag)
                    print("Error:", error_x, error_y, error_theta)
                    print("Speed:", v_x, v_y, w)
                    print()

                # Create a Twist message for control
                control_twist = Twist()
                control_twist.linear.x = v_x
                control_twist.linear.y = v_y
                control_twist.angular.z = w

                # Publish the control_twist to control the robot
                ebot_controller.twist_publisher.publish(control_twist)

                # this if below let the index increment only if you reach the desired goal
                goal_error = math.sqrt(error_x**2 + error_y**2)
                if goal_error < 0.1 and abs(error_theta) < 0.1:
                    ebot_controller.get_logger().info(f'Reached Goal: x:{x_goal}, y:{y_goal}, theta:{theta_goal}\n')

                    ############     DO NOT MODIFY THIS       #########
                    ebot_controller.index += 1
                    if ebot_controller.flag == 1 :
                        ebot_controller.index = 0
                    ebot_controller.send_request(ebot_controller.index)
                    ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(ebot_controller)
    
    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
