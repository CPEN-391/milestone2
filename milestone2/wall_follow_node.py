import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from queue import Queue
from math import *

class WallFollower(Node):
    #Adjustable variables
    kp = 100
    ki = 0
    kd = 75
    integralSize = 20
    clockwise = False

    integralQueue = Queue()
    integralError = 0
    lastError = 0
    thisError = 0
    lastSteeringAngle = 0
    lastSpeed = 0
    latest_odom = None


    
    def __init__(self):
        super().__init__('wall_follow_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive_raw', 10)

        self.publisher  # prevent unused variable warning
        self.subscription  # prevent unused variable warning
        
        

    def odom_callback(self, msg):
        """
        Processes incoming Odometry messages.
        """
        self.latest_odom = msg



    def scan_callback(self, scan_masg):

        """
        Processes incoming LaserScan messages.
        """

        # Pick angles based on direction
        if(self.clockwise):
            lidar180 = scan_masg.ranges[180]  # dist at 180 deg
            lidar70 = scan_masg.ranges[620]  # dist at 70 deg
            lidar0 = scan_masg.ranges[900]  # dist at 0 deg     
        else:
            lidar180 = scan_masg.ranges[900]  # dist at 180 deg
            lidar70 = scan_masg.ranges[460]  # dist at 70 deg
            lidar0 = scan_masg.ranges[180]  # dist at 0 deg

        self.calculateError(radians(70), lidar0, lidar70, lidar180)
        self.vehicleControl()

        # Log values for debugging
        self.get_logger().info(
            f'\nLidar 0 (m): {lidar0:.2f}'
            f'\nLidar 70 (m): {lidar70:.2f}'
            f'\nLidar 180 (m): {lidar180:.2f}'
            f'\nError Term (rad): {self.thisError:.2f}'
        )
        
    
    
    
    def calculateError(self, theta, lidar0, lidarTheta, lidar180, L = 0.2):
        
        # Lookahead distance L based on speed TODO: its hard to get the speed and time right.
        #L = self.lastSpeed * 0.05 # relying on a callback time of 50 ms...
        #self.get_logger().info(f'Lookahead distance L based on speed: {L:.2f} m')

        
        alpha = atan((lidarTheta*cos(theta) - lidar0) / (lidarTheta*sin(theta)))

        # distance to wall now
        Dt = lidar0 * cos(alpha)

        # projected distance after lookahead
        Dt_L = Dt + L * sin(alpha)

        # error: positive if too far, negative if too close
        self.thisError = (lidar0 + lidar180)/2 - Dt_L
    
    def proportionalControl(self):
        return self.kp * self.thisError
    
    def derivativeControl(self):
        return self.kd * (self.thisError - self.lastError)
    
    def integralControl(self):
        self.integralError = self.integralError + self.thisError  # Fixed: add assignment
        self.integralQueue.put(self.thisError)
        if(int(self.integralQueue.qsize()) >= self.integralSize):
            self.integralError = self.integralError - self.integralQueue.get()  # Fixed: add assignment
        return self.ki * self.integralError  # Added return value


    def vehicleControl(self):
        # PID control law
        control = radians(self.proportionalControl() + self.derivativeControl() + self.integralControl())

        # In radians, max steering angle is capped at 0.42
        max_angle = 0.4189  # radians ≈ 24°
        newSteeringAngle = max(min(control, max_angle), -max_angle)

        max_speed = 3  # m/s
        min_speed = 1.0  # m/s
        turn_factor = abs(newSteeringAngle) / max_angle  # 0.0 (straight) → 1.0 (max turn)
        newSpeed = max_speed - turn_factor * (max_speed - min_speed)

        self.lastSteeringAngle = newSteeringAngle 
        self.lastSpeed = newSpeed

        # Publish steering angle and speed.
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = newSteeringAngle
        drive_msg.drive.speed = newSpeed
        self.publisher.publish(drive_msg)

        self.get_logger().info(f"Error: {self.thisError:.2f}, Control: {control:.2f}, Steering Angle: {newSteeringAngle:.2f}")
        self.get_logger().info(f"Speed: {self.lastSpeed:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollower()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
