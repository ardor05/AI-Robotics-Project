#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.15
STOP_DISTANCE = 0.1
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class ObstacleRightWall():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle_right_wall()
        
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)
        samples_view = 1  # Set to 1 for simplicity
        
        if samples_view > samples:
            samples_view = samples

        if samples_view == 1:
            scan_filter.append(scan.ranges[0])
        else:
            left_lidar_samples_ranges = -(samples_view // 2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view // 2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter

    def obstacle_right_wall(self):
        twist = Twist()
        turtlebot_moving = True
        run_away = False
        right_wall_following = False

        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)

            if min_distance < SAFE_STOP_DISTANCE:
                if turtlebot_moving:
                    # Move away from the obstacle
                    twist.linear.x = -LINEAR_VEL  # Reverse
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = False
                    run_away = True
                    right_wall_following = False
                    rospy.loginfo('Run away from the obstacle!')
            
            else:
                if run_away:
                    # Perform a right turn after running away
                    twist.linear.x = 0.0
                    twist.angular.z = -2.5  # Adjust the angular velocity as needed
                    self._cmd_pub.publish(twist)
                    rospy.sleep(1.0)  # Adjust the sleep duration as needed for the turn

                    # Continue moving forward
                    twist.linear.x = LINEAR_VEL
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = True
                    run_away = False
                    right_wall_following = False
                    rospy.loginfo('Right turn after running away!')
                
                elif not right_wall_following:
                    # Start right wall following
                    right_wall_following = True
                    rospy.loginfo('Starting Right Wall Following!')

                else:
                    # Continue right wall following behavior
                    twist.linear.x = LINEAR_VEL
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = True
                
                    rospy.loginfo('Distance of the obstacle : %f', min_distance)

def main():
    rospy.init_node('turtlebot3_obstacle_right_wall')
    try:
        obstacle_right_wall = ObstacleRightWall()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

