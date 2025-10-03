import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import math


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

         # Store latest odometry and scan data
        self.latest_odom = None
        self.latest_scan = None
        self.latest_drive_raw = None
        self.linear_x_speed = 0
        
        # Subscribers -------------
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', # topic
            self.scan_callback,
            10 # QoS, Quality of Service, limits the amount of queued msgs
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            '/drive_raw', # subscribe to drive topic from car control
            self.drive_raw_callback,
            10
        )

        # Publishers -------------
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive', # topic
            10 
        ) 




    def scan_callback(self, msg: LaserScan):
        """
        Processes incoming LaserScan messages.
        """
        self.latest_scan = msg
        self.check_for_collision() 


    def odom_callback(self, msg: Odometry):
        """
        Processes incoming Odometry messages.
        """
        self.latest_odom = msg

    
    def drive_raw_callback(self, msg: AckermannDriveStamped):
        """
        Processes incoming driving control messages.
        """
        self.latest_drive_raw = msg




    def check_for_collision(self):
 
        # Ensure we have the necessary data
        if self.latest_scan is None:
            return
        if self.latest_odom is None:
            self.get_logger().info('No odom yet, assuming 1 m/s')
            self.linear_x_speed = 1
        else:
            self.linear_x_speed = self.latest_odom.twist.twist.linear.x
            self.get_logger().info(f'Getting odom speed: {self.linear_x_speed}')
  

        # Collision logic here
        time_to_collision = self.calculate_time_to_collision(self.latest_odom, self.latest_scan)
        
        # Check for incoming collisions (TODO: define threshold)
        if time_to_collision < 0.1:
            self.publish_stop()
            self.get_logger().warn("TTC critical, stopping")
        elif time_to_collision < 0.5:
            self.get_logger().warn("TTC low, braking")
            self.publish_brake(0.5)
        elif self.latest_drive_raw is None:
            self.get_logger().info("No drive command received yet, not publishing")
        else:
            self.publisher.publish(self.latest_drive_raw)


    

    def calculate_time_to_collision(self, odom_msg:Odometry, scan_msg: LaserScan):
        
        range_min_tts = float('inf')
        car_radius = 0.30

        scan_window = 30 # deg to each side
        left_scan_border = 540 + scan_window * 4
        right_scan_border = 540 - scan_window * 4


        window = left_scan_border - right_scan_border # window of lidar is +- 30 degrees from front of car

        if scan_msg.ranges: # check that we have ranges data
            for i in range(window):
                curr_range = scan_msg.ranges[i+right_scan_border] - car_radius

                if curr_range > scan_msg.range_min and curr_range < scan_msg.range_max:

                    # calculate relevant parametes for calculating TTS
                    beam_angle = (-540 + i) * 4 # get the beam angle in degrees from vehicles x axis
                    linear_speed = self.linear_x_speed
                    range_rate = linear_speed * math.cos(beam_angle * 2 * math.pi / 360)
                    
                    # Ignore range rates of <= 0
                    if range_rate <= 0:
                        time_to_collision = float('inf')
                    else:
                        time_to_collision = curr_range / range_rate

                    # return only the minimum TTS for the range
                    if time_to_collision < range_min_tts:
                        range_min_tts = time_to_collision
                    

        self.get_logger().info(f'Time to collision is: {range_min_tts}')
        return range_min_tts 
            

    # Emergency stop the node immediately
    def publish_stop(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.publisher.publish(msg)

        self.get_logger().warn("Car stop published, shutting down node")
    
        rclpy.shutdown()

    
    # Brake the node gently more gently
    def publish_brake(self, brake_factor=0.5):
        current_speed = self.latest_odom.twist.twist.linear.x
        

        msg = AckermannDriveStamped()
        msg.drive.speed = current_speed * brake_factor


        self.get_logger().info("Braking, publishing half speed")
        self.publisher.publish(msg)

        


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode() 
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
