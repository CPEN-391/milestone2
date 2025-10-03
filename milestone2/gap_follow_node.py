import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from math import *


class GapFollowing(Node):
    # GAP FOLLOWER VARIABLES
    car_width = 0.3  # meters
    min_gap_depth = 2.0  # meters
    min_gap_width = 40 # indeces
    safety_buffer = 0.2  # meters
    disparity_safety_indeces = 8 # indeces
    safety_bubble_radius = 0.3 # meter

    def __init__(self):
        super().__init__('gap_follow_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive_raw', 10)
        self.get_logger().info("GapFollowing node initialized and subscribed to /scan")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received LaserScan with {len(msg.ranges)} ranges")
        
        # Calculate the steering angle
        steering_angle = self.gap_following(msg.ranges, self.min_gap_width, self.min_gap_depth)
        self.get_logger().info(f"Got steering angle {steering_angle} radians")

        if steering_angle is not None:
            self.vehicleControl(steering_angle)
            self.get_logger().info(f"Steering Angle Command: {steering_angle:.2f} rad")
        else:
            self.get_logger().warn("No valid gap found!")


    # Calculate the steering angle
    def gap_following(self, lidar_ranges, min_gap_width, min_gap_depth):

        bubbled_lidar = self.bubble_nearest_point(lidar_ranges)
        
        deep_scans = list(filter(lambda x: x[1] > min_gap_depth, enumerate(lidar_ranges)))
        deep_scan_indeces = [i for i, val in deep_scans]
        
        # find first all gaps with at leat 10 consecutive deep scans.
        gaps = self.find_gaps(deep_scan_indeces, min_gap_width)
        safe_gaps = self.safe_gaps(bubbled_lidar, gaps)    

        # Choose the widest or the deepest gap
        #chosen_gap = self.choose_deepest_gap(bubbled_lidar, safe_gaps)
        chosen_gap = self.choose_widest_gap(bubbled_lidar, safe_gaps)
    
        if chosen_gap is not None:
            middle_index = chosen_gap[len(chosen_gap) // 2]
            steering_angle = self.steeringIndexToRadians(middle_index)
        else:
            # No safe gaps found, take safe fallback
            steering_angle = 0.0  # go straight
            self.get_logger().warn("No safe gaps found! Going straight.")
        return steering_angle # in radians.

    
    
    #------------------------- START HELPER FUNCTIONS --------------------------------------
    # cuts off the edge of gaps to avoid cutting corners.
    def disparity_extender(self, gap, indeces_to_cut:int):
        return gap[indeces_to_cut : -indeces_to_cut] # return trimmed gap
        
    
    def gap_width(self, lidar_ranges, gap):
        gap_start = gap[0]
        gap_middle = gap[ len(gap) // 2 ]
        gap_end = gap[-1]

        dist1 = lidar_ranges[gap_start]
        dist2 = lidar_ranges[gap_end]

        angle_deg = (len(gap)) * 4
        angle_rad = angle_deg * pi/180

        return sqrt(dist1**2 + dist2**2 - 2 * dist1 * dist2 * cos(angle_rad))
    
    def find_gaps(self, deep_scan_indeces, min_width):
        gaps = []
        temp_gap = [deep_scan_indeces[0]]

        for i in range(1, len(deep_scan_indeces)):
            if deep_scan_indeces[i] == deep_scan_indeces[i-1] + 1:
                temp_gap.append(deep_scan_indeces[i])
            else:
                gap_to_append = self.disparity_extender(temp_gap, self.disparity_safety_indeces)
                
                if len(gap_to_append) >= min_width:
                    gaps.append(gap_to_append)
                temp_gap = [deep_scan_indeces[i]]

        # Check the last sequence
        if len(temp_gap) >= min_width:
            gaps.append(temp_gap)
        
        return gaps # returns [[22,23,24,25],[44,45,46,47,48,49]]
    
    def steeringIndexToRadians(self, index):
        angle_deg = (-540 + index) / 4 # index 0 = 135 deg, index 1080 = -135 deg
        angle_rad = angle_deg * pi / 180
        return angle_rad
    
    def choose_deepest_gap(self, lidar_ranges, safe_gaps):
        if not safe_gaps:
            return None  # No safe gaps

        # Pair each gap with the depth at its middle
        gap_middle_depths = [(gap, lidar_ranges[gap[len(gap)//2]]) for gap in safe_gaps]

        # Select the gap whose middle has the maximum depth
        deepest_gap = max(gap_middle_depths, key=lambda x: x[1])[0]

        return deepest_gap
    
    def choose_widest_gap(self, lidar_ranges, safe_gaps):
        if not safe_gaps:
            return None  # no safe gaps
        
        widest_gap = max(safe_gaps, key = lambda gap: self.gap_width(lidar_ranges, gap))
        return widest_gap
   
    def safe_gaps(self, lidar_ranges, gaps):
        safe_gaps = []
        for gap in gaps:
            gap_width = self.gap_width(lidar_ranges, gap)
            if gap_width > self.car_width + 2 * self.safety_buffer:
                safe_gaps.append(gap)
        return safe_gaps
    

    # avoid nearest obstacle
    def bubble_nearest_point(self, lidar_ranges):
        nearest_index, distance = min(enumerate(lidar_ranges), key=lambda x: x[1])
        
        #angle_rad = 2 * asin(2 * self.safety_bubble_radius / (2 * distance))
        ratio = min(1.0, self.safety_bubble_radius / distance)
        angle_rad = 2 * asin(ratio)

        no_indeces = ceil((angle_rad * 180/pi) * 4) # deg * 4 = indeces
        
        start = max(0, nearest_index - no_indeces)
        end   = min(len(lidar_ranges), nearest_index + no_indeces + 1)
        for i in range(start, end):
            lidar_ranges[i] = 0.0
        
        return lidar_ranges



    #------------------------- END HELPER FUNCTIONS --------------------------------------

    


            
    def vehicleControl(self, angle):
        max_angle = 0.4189  # radians ≈ 24°
        newSteeringAngle = max(min(angle, max_angle), -max_angle)

        max_speed = 1.5  # m/s
        min_speed = 0.5  # m/s
        turn_factor = abs(newSteeringAngle) / max_angle  # 0.0 (straight) → 1.0 (max turn)
        newSpeed = max_speed - turn_factor * (max_speed - min_speed)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = newSteeringAngle
        drive_msg.drive.speed = newSpeed
        self.publisher.publish(drive_msg)

        self.get_logger().info(
            f"Published Steering: {newSteeringAngle:.2f} rad, "
            f"Speed: {drive_msg.drive.speed:.2f} m/s"
        )


def main(args=None):
    rclpy.init(args=args)
    gap_follow_node = GapFollowing()
    rclpy.spin(gap_follow_node)
    gap_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
