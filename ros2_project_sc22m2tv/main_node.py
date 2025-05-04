import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from math import sin, cos
from rclpy.callback_groups import ReentrantCallbackGroup
from enum import Enum

# State robot is currently in
class Mode(Enum):
    EXPLORING = 1
    APPROACHING_BLUE = 2
    DONE = 3

# Robot node definition
class SimpleRobot(Node):
    def __init__(self):
        super().__init__('simple_robot')
        # Image subscription
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.latest_image = None

        # Nav setup
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=ReentrantCallbackGroup())
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Generate grid waypoints
        self.waypoints = self.generate_grid_waypoints(
            x_min=-9.0,
            x_max=7.0,
            y_min=-13.0,
            y_max=5.0,
            steps=5
        )
        self.current_goal_index = 0
        self.goal_in_progress = False
        self.current_goal_handle = None

        self.mode = Mode.EXPLORING  # Default mode

    # Image processing func
    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # Convert to cv2 format
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Convert images to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        colors = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'green': ([50, 100, 100], [70, 255, 255]),
            'blue': ([110, 100, 100], [130, 255, 255])
        }

        for color, (lower, upper) in colors.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))  # Apply mask
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # Find countours
            # Check countours
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 100:
                    x, y, w, h = cv2.boundingRect(cnt)  # Bounding box
                    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    cv2.putText(image, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    # If blue blox is large enough then switch mode 
                    if color == 'blue' and area > 1000 and self.mode == Mode.EXPLORING:
                        self.get_logger().info("Blue box detected")
                        self.mode = Mode.APPROACHING_BLUE
                        if self.goal_in_progress:
                            self.cancel_current_goal()

        # Blue box approaching logic
        if self.mode == Mode.APPROACHING_BLUE:
            mask = cv2.inRange(hsv, np.array([110, 100, 100]), np.array([130, 255, 255]))
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest = max(contours, key=cv2.contourArea)  # Find largest contour
                area = cv2.contourArea(largest)

                if area > 100:
                    M = cv2.moments(largest)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])  # FInd largest blue contour
                        img_center = image.shape[1] // 2
                        error = cx - img_center
                        cmd = Twist()

                        # Move towards robot
                        if abs(error) > 50:
                            cmd.angular.z = -0.002 * error
                        else:
                            cmd.linear.x = 0.05
                        self.cmd_vel_pub.publish(cmd)

                        # If area is large enough then stop
                        if area > 10000:
                            self.cmd_vel_pub.publish(Twist())
                            self.get_logger().info("Blue box reached (within ~1m). Task complete.")
                            self.mode = Mode.DONE
                            self.goal_in_progress = False
                            return

        self.latest_image = image
        if self.mode == Mode.EXPLORING and not self.goal_in_progress:  # Send goal if still exploring
            self.send_next_goal()

    # Func to send next waypoint
    def send_next_goal(self):
        if self.mode != Mode.EXPLORING:  # Send next waypoint if not moving already
            self.get_logger().info(f"Failed to send next goal. Current mode is {self.mode.name}")
            return

        # Loop back to beginning if all waypoints visited
        if self.current_goal_index >= len(self.waypoints):
            self.current_goal_index = 0
        x, y, theta = self.waypoints[self.current_goal_index]
        self.get_logger().info(f"Sending goal {self.current_goal_index + 1}: x={x}, y={y}")

        # Define next goal to send
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = cos(theta / 2.0)
        
        # Send next goal
        self.goal_in_progress = True
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    # Func that handles response after sending goal
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:  # If goal rejected
            self.get_logger().warning('Goal rejected. Skipping to next.')
            self.goal_in_progress = False
            self.current_goal_index += 1
            self.send_next_goal()
            return
        
        # Goal accepted
        self.get_logger().info('Goal accepted.')
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # Func that handles goal completion
    def result_callback(self, future):
        self.get_logger().info('Goal reached.')
        self.goal_in_progress = False
        if self.mode == Mode.EXPLORING:  # If still exploring the map
            self.current_goal_index += 1
            self.send_next_goal()
        else:  # Already seen blue box
            self.get_logger().info(f"Skipping next goal because mode is {self.mode.name}")

    # Func to cancel goal when blue box is seen
    def cancel_current_goal(self):
        self.get_logger().info("Cancelling current Nav2 goal.")
        if self.current_goal_handle:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            def on_cancel_complete(future):
                self.get_logger().info("Goal cancelled.")
                self.goal_in_progress = False
                self.current_goal_handle = None
            cancel_future.add_done_callback(on_cancel_complete)
        else:
            self.goal_in_progress = False
            self.current_goal_handle = None

    # Func to generate grid-pbased waypoints
    def generate_grid_waypoints(self, x_min, x_max, y_min, y_max, steps):
        waypoints = []
        # Evenly space waypoints depending on x and y values
        x_values = np.linspace(x_max, x_min, steps)
        y_values = np.linspace(y_min, y_max,  steps)

        flip = False  # Create snaking pattern
        for y in y_values:
            row = []
            for x in x_values:
                row.append((x, y, 0.0))  # Add waypoints to each row
            if flip:
                row.reverse()  # Reverse order to make snaking pattern
            waypoints.extend(row)
            flip = not flip
        return waypoints

def main(args=None):
    # Initialize robot
    rclpy.init(args=args)
    node = SimpleRobot()

    # ROS loop
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.latest_image is not None:  # Display most recent camera frame
                cv2.imshow('Camera View with Detections', node.latest_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit on 'q'
                    break
    # Cleanup
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
