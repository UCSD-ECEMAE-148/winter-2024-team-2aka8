import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32, Int32, Float32MultiArray, Int32MultiArray, MultiArrayDimension, MultiArrayLayout
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
import time
import os
from openai import OpenAI, OpenAIError
import json
import threading
import base64
import subprocess

from ast import arg
from turtle import pos
import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped



# Nodes in this program
NODE_NAME = 'chatgpt_drive_node'

# Topics subcribed/published
CAMERA_TOPIC_NAME = '/camera/color/image_raw'
ACTUATOR_TOPIC_NAME = '/teleop'
LIDAR_TOPIC_NAME = '/scan'

api_key = "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"
client = OpenAI(api_key=api_key)
manage_py_path = '/home/projects/ros2_ws/mycar/manage.py'

SYSTEM_MESSAGE_VISION = (
    "This ChatGPT instance controls a simulated robot car designed to follow user commands for "
    "movement and navigation. It can process one command:"
    " direct drive commands specifying speed and steering angle. If provided with a task, YOU MUST describe enough of the image"
    "for the next model to properly make commands."
    "You cannot function call as a vision model, so create an action plan. For example, if the user asks you to go"
    "to an object, describe about where the object is. the next model will decide actual settings."
    "Listed are the tools you can use. Be simple, as a function calling gpt3.5 model will"
    "interpret your instruction and command. MAKE SURE YOUR PATH STARTS AT 0,0. Your reference area is around 10x10 meters"
)

TOOLER_NOTE = ( 
    "What you just read is what gpt4 with vision recieved. Your job as gpt4 with function calling is now to take gpt4V's response"
    "and turn it into function calling. Also use the lidar data at your disposal to adjust"
    "gpt4V's plan settings. GPT4 vision cannot function call but you can. If the prompts"
    "are irrelevant to any functions, just say no functions called."
)

tools = [
    {
        "type": "function",
        "function": {
            "name": "send_drive_command",
            "description": "Send drive commands to the robot",
            "parameters": {
                "type": "object",
                "properties": {
                    "speed": {
                        "type": "number",
                        "description": "The speed at which the robot should move. Range is from 0.5 to 3",
                    },
                    "steering_angle": {
                        "type": "number",
                        "description": "The steering angle for the robot's movement",
                    },
                    "timeout": {
                        "type": "number",
                        "description": "The duration for which the robot should move in seconds",
                    }
                },
                "required": ["speed", "steering_angle", "timeout"],
            },
        }
    },
    {
        "type": "function",
        "function": {
            "name": "format_position_command",
            "description": "Format positions into robot control commands",
            "parameters": {
                "type": "object",
                "properties": {
                    "positions": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "px": {"type": "number", "description": "Position in x-coordinate"},
                                "py": {"type": "number", "description": "Position in y-coordinate"},
                                "throttle": {"type": "number", "description": "Throttle value from 0 to 1, usually 0.5"},
                            },
                            "required": ["px", "py", "throttle"]
                        },
                        "description": "Array of positions and orientations in degrees and meters"
                    }
                },
                "required": ["positions"]
            },
        }
    },
    
    
]


class ChatgptDriveSubpub(Node):
    def __init__(self):
        """
        Initializes the ChatGPTDriveNode class.

        This class represents a ROS node that integrates the Donkey Car platform with ChatGPT for autonomous driving.
        It sets up the necessary subscribers, publishers, and parameters for controlling the car and interacting with ChatGPT.
        """
        super().__init__(NODE_NAME)

        # Constants
        self.QUEUE_SIZE = 10
        self.frame_id = 'base_link'

        # Initialize
        self._init_ros_parameters()
        self._init_camera_subscription()
        self._init_drive_publisher()
        self._init_chatgpt_interaction()
        #self._init_path_publisher()
        self._init_lidar_subscription()

        self.get_logger().info("end init")

    def _init_ros_parameters(self):
        """Initialize ROS parameters."""
        self.declare_parameters(
            namespace='',
            parameters=[
                ('live_camera_feed', 0),
                ('Ts', 0.05)
            ]
        )
        self.live_camera_feed = self.get_parameter('live_camera_feed').value
        self.Ts = self.get_parameter('Ts').value  # controller sample time
        self.get_logger().info(
            f'\n live_camera_feed: {self.live_camera_feed}'
            f'\n Ts: {self.Ts}'
        )

    def _init_camera_subscription(self):
        """Initialize camera subscription."""
        self.camera_thread = MutuallyExclusiveCallbackGroup()
        self.camera_actuator_subpub = self.create_subscription(
            Image, CAMERA_TOPIC_NAME, self.camera_callback, self.QUEUE_SIZE, 
            callback_group=self.camera_thread
        )
        self.bridge = CvBridge()
        self.image_frame = None

    def _init_drive_publisher(self):
        """Initialize drive publisher."""
        self.drive_pub = self.create_publisher(AckermannDriveStamped, ACTUATOR_TOPIC_NAME, self.QUEUE_SIZE)
        self.drive_cmd = AckermannDriveStamped()
        self.current_time = self.get_clock().now().to_msg()

    def _init_chatgpt_interaction(self):
        """Initialize ChatGPT interaction."""
        self.chat_input_sub = self.create_subscription(
            String, 'chat_input', self.chat_input_callback, 10
        )
        self.chat_output_pub = self.create_publisher(String, "chat_output", 10)

    def _init_path_publisher(self):
        """Initialize path publisher."""
        self.path_publisher_ = self.create_publisher(Path, '/trajectory', qos_profile_sensor_data)
        self.declare_parameter("frame_id", "map")  # Defaulting to 'map', change as needed
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
    
    def _init_lidar_subscription(self):
        self.laser_subscriber = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.lidar_callback, self.QUEUE_SIZE)

        # Lidar info
        self.lidar_properties_set = False
        self.default_viewing_angle = 360
        self.default_front_degree_angle = 0
        self.default_right_degree_angle = 90
        self.default_left_degree_angle = 270
        self.range_max = None
        self.range_min = None
        self.num_scans = None
        self.angle_increment = None
        self.angle_min_radians = None
        self.angle_max_radians = None
        self.scan_ranges = None
        self.formatted_scan_data = "Empty"

        # ROS parameters (setttng default values)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('viewing_angle', self.default_viewing_angle),
                ('front_degree_angle', self.default_front_degree_angle),
                ('right_degree_angle', self.default_right_degree_angle),
                ('left_degree_angle', self.default_left_degree_angle),
                ('live_laser_feed', 0)
            ])

        # Update ROS parameters from config/launch file
        self.viewing_angle = self.get_parameter('viewing_angle').value
        self.front_degree_angle = self.get_parameter('front_degree_angle').value
        self.right_degree_angle = self.get_parameter('right_degree_angle').value
        self.left_degree_angle = self.get_parameter('left_degree_angle').value
        self.live_laser_feed = self.get_parameter('live_laser_feed').value

    def lidar_callback(self, data):
        if not self.lidar_properties_set:
            self.range_max = data.range_max
            self.range_min = data.range_min
            self.num_scans = len(np.array(data.ranges))
            self.angle_increment = data.angle_increment
            self.angle_min_radians = data.angle_min
            self.angle_max_radians = data.angle_max
            self.lidar_properties_set = True
        
        scan_ranges = np.array(data.ranges)
        self.scan_ranges = np.nan_to_num(scan_ranges, neginf=self.range_max, posinf=self.range_max, nan=self.range_max)

        # printing distances from lidar measurements
        if self.live_laser_feed:
            self.get_logger().info(
                f'\n scan_ranges: {self.scan_ranges}'
                )

        # Calculate corresponding degrees for each scan range
        angles_degrees = np.degrees(np.arange(self.angle_min_radians, self.angle_max_radians, self.angle_increment))
        if len(angles_degrees) > len(self.scan_ranges):
            angles_degrees = angles_degrees[:len(self.scan_ranges)]  # Ensure angle and range arrays match

        # Combine ranges with their corresponding angles
        scan_data_with_angles = np.vstack((angles_degrees, self.scan_ranges)).T

        # Format the scan data for sending. Here, we send raw scan data.
        self.formatted_scan_data = "; ".join([f"Ang: {angle:.2f}°, Dist: {distance:.2f} m" for angle, distance in scan_data_with_angles])
        #self.get_logger().info(self.formatted_scan_data)


        


    def camera_callback(self, data):
        # Image data
        self.image_frame = self.bridge.imgmsg_to_cv2(data)

    def chat_input_callback(self, msg):
        user_input = msg.data
        self.get_logger().info(
            f'\n User: {user_input}'
        )
        self.process_chatgpt_command(user_input)
        

    def process_chatgpt_command(self, prompt):
        try:
            # Process image
            #self.get_logger().info("abt to convert img")
            _, img = cv2.imencode('.jpg', self.image_frame)
            base64_img = base64.b64encode(img).decode('utf-8')
            #self.get_logger().info("abt to get resp")

            vision_response =client.chat.completions.create(
                model="gpt-4-vision-preview",
                messages=[{
                    "role": "system",
                    "content": json.dumps({"message": SYSTEM_MESSAGE_VISION + str(tools)})
                }, {
                    "role": "user",
                    "content": [
                        {
                            "type": "text", 
                            "text": prompt
                        },
                        {
                            "type": "image_url", 
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_img}",
                                "detail": "high"
                            }
                        }
                    ]
                }],
                max_tokens=300,
            )

            vision_response_text = vision_response.choices[0].message.content.strip()
            self.get_logger().info("GPT Vision Response: %s" % vision_response_text)

            tooler_response = client.chat.completions.create(
                model="gpt-4-turbo-preview",
                messages=[{
                    "role": "system",
                    "content": json.dumps({"message": SYSTEM_MESSAGE_VISION + TOOLER_NOTE, "tools": tools})
                }, {
                    "role": "user",
                    "content": [
                        {
                            "type": "text", 
                            "text": prompt
                        },
                    ]
                    
                }, {
                    "role": "assistant",
                    "content": [
                        {
                            "type": "text", 
                            "text": vision_response_text
                        },
                    ]
                },   
                ],
                tools=tools,
            )

            self.get_logger().info("tooler time")
            
            # Assuming the response includes a function call to 'send_drive_command'
            if tooler_response.choices[0].message.tool_calls:
                if tooler_response.choices[0].message.tool_calls[0].function.name == "send_drive_command":
                    self.get_logger().info("in tool call area")

                    
                    speed = json.loads(tooler_response.choices[0].message.tool_calls[0].function.arguments)["speed"]
                    self.get_logger().info(str(speed))

                    speed = float(speed)

                    steering_angle = json.loads(tooler_response.choices[0].message.tool_calls[0].function.arguments)["steering_angle"]
                    self.get_logger().info(str(steering_angle))

                    steering_angle = float(steering_angle)

                    timeout = json.loads(tooler_response.choices[0].message.tool_calls[0].function.arguments)["timeout"]
                    self.get_logger().info(str(timeout))

                    timeout = float(timeout)

                    self.send_drive_command(speed, steering_angle, timeout)
                    self.get_logger().info("doing drive command")

                elif tooler_response.choices[0].message.tool_calls[0].function.name == "format_position_command":
                    positions = json.loads(tooler_response.choices[0].message.tool_calls[0].function.arguments)["positions"]
                    #self.format_position_command(positions)
                    self.export_path_csv(positions)
                    #self.donkey_drive()

            else:
                # Get the plain text response
                chatgpt_tooler_response = tooler_response.choices[0].message.content.strip()

                # Send the response to the terminal
                self.get_logger().info("ChatGPT Functioner: %s" % chatgpt_tooler_response)

        except OpenAIError as e:
            self.get_logger().error(f"Failed to get response from ChatGPT: {e}")

    def export_path_csv(self, positions):
        # Export path to CSV
        df = pd.DataFrame(positions)

        #i = 1
        #while os.path.exists(f'/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_basics2_pkg/ucsd_robocar_basics2_pkg/path_data({i}).csv'):
        #    i += 1

        df.to_csv(f'/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_basics2_pkg/ucsd_robocar_basics2_pkg/donkey_paths/chat_gps.csv', index=False, header=False)

    def donkey_drive(self, retries=1, delay=2):
        attempt = 0
        while attempt <= retries:
            try:
                result = subprocess.run(['python3', manage_py_path, 'drive'], capture_output=True, text=True)
                if result.returncode == 0:
                    self.get_logger().info("Command executed successfully.")
                    self.get_logger().info(result.stdout)
                    break
                else:
                    raise Exception(f"Command failed with return code {result.returncode}. Error: {result.stderr}")
            except Exception as e:
                self.get_logger().error(f"Attempt {attempt+1} failed: {e}")
                if attempt < retries:
                    self.get_logger().info(f"Retrying in {delay} seconds...")
                    time.sleep(delay)
                attempt += 1
        else:
            self.get_logger().error("All attempts to run the command have failed.")


    def format_position_command(self, positions):
        # Initialize the path message
        self.path_msg.poses = []  # Clear existing poses
        for position in positions:
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = float(position['px'])
            pose_msg.pose.position.y = float(position['py'])
            pose_msg.pose.position.z = float(position['pz'])
            pose_msg.pose.orientation.x = float(position['qx'])
            pose_msg.pose.orientation.y = float(position['qy'])
            pose_msg.pose.orientation.z = float(position['qz'])
            pose_msg.pose.orientation.w = float(position['qw'])
            self.path_msg.poses.append(pose_msg)
    
        # After constructing the path, publish it
        self.publish_path()

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher_.publish(self.path_msg)


    def send_drive_command(self, speed, steering_angle, timeout):
        def publish_command():
            # Calculate the end time based on the current time and the specified timeout
            end_time = time.time() + timeout

            while time.time() < end_time:
                drive_msg = AckermannDriveStamped()
                drive_msg.drive.speed = float(speed)
                drive_msg.drive.steering_angle = steering_angle
                self.drive_pub.publish(drive_msg)
                time.sleep(self.Ts)  # Wait for 0.05 seconds before publishing the next command

            # Optionally, call stop_drive_command after the loop finishes
            self.stop_drive_command()

        # Cancel any existing timer or thread
        if hasattr(self, 'drive_command_thread') and self.drive_command_thread.is_alive():
            self.drive_command_thread.cancel()  # Ensure this method exists and is implemented to safely stop the thread

        # Create and start the thread
        self.drive_command_thread = threading.Thread(target=publish_command)
        self.drive_command_thread.start()

    def stop_drive_command(self):
        # Stop the vehicle
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(drive_msg)






def main(args=None):
    rclpy.init(args=args)
    chatgpt_drive_subpub = ChatgptDriveSubpub()
    try:
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(chatgpt_drive_subpub)
        try:
            executor.spin()
        finally:
            chatgpt_drive_subpub.get_logger().info(f'Shutting down {NODE_NAME}...')
            chatgpt_drive_subpub.drive_cmd.header.stamp = chatgpt_drive_subpub.current_time
            chatgpt_drive_subpub.drive_cmd.header.frame_id = chatgpt_drive_subpub.frame_id
            chatgpt_drive_subpub.drive_cmd.drive.speed = 0.0
            chatgpt_drive_subpub.drive_cmd.drive.steering_angle = 0.0
            chatgpt_drive_subpub.drive_pub.publish(chatgpt_drive_subpub.drive_cmd)
            time.sleep(1)
            chatgpt_drive_subpub.get_logger().info(f'{NODE_NAME} shut down successfully.')
            executor.shutdown()
            chatgpt_drive_subpub.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()