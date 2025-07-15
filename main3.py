import cv2
from setup import setup
from webcam1 import TrafficSignDetector
from Util import Global
from Controller import speed_controller
from Controller import lateral_controller as lc
from Util import Referans,State
import sys
import glob
import detection.detect as dt1 #detect yazıyodu burda
import detection.cone_detector as cd  # Add cone detection
import detection.torch_detector as torch_det  # Add simple YOLOv8 detector
import math  # Add math import for cone distance calculation
import pygame  # Add pygame import for manual control
try:
    sys.path.append(glob.glob('../../carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg')[0])
except IndexError:
    pass
import carla  
import threading
import time
import numpy as np

from FinalPathGenerator import Levha
from FinalPathGenerator import setLabel
from FinalPathGenerator import PathGenerator as pg
from FinalPathGenerator import Main
from Util import Global
from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap
from Controller.waypoint_interpolation import WaypointInterpolation
import detection.detect as dt1
import socket
import struct
import pickle

# ROS2 imports (optional - only if ROS2 is available)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField, Imu, CameraInfo
    from std_msgs.msg import Int32, String, Float32
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Header
    import tf_transformations
    ROS2_AVAILABLE = True
except ImportError:
    print("Warning: ROS2 not available. ROS2 publishing will be disabled.")
    ROS2_AVAILABLE = False

# ROS2 Node class (only if ROS2 is available)
if ROS2_AVAILABLE:
    class CarlaROS2Node(Node):
        def __init__(self):
            super().__init__('carla_data_recorder')
            
            # Publishers for sensor data
            self.lidar_publisher = self.create_publisher(PointCloud2, '/points_raw', 10)
            self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
            self.imu_publisher = self.create_publisher(Imu, '/imu_raw', 10)
            self.calib_pub = self.create_publisher(CameraInfo, '/camera_info', 10)
            
            # Publishers for vehicle status and control data
            self.traffic_sign_pub = self.create_publisher(String, '/traffic_sign', 10)
            self.vehicle_status_pub = self.create_publisher(String, '/vehicle_status', 10)
            self.cone_detection_pub = self.create_publisher(String, '/cone_detection', 10)
            self.vehicle_control_pub = self.create_publisher(String, '/vehicle_control', 10)
            self.steering_pub = self.create_publisher(Float32, '/steering_angle', 10)
            self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
            self.brake_pub = self.create_publisher(Float32, '/brake', 10)
            self.speed_pub = self.create_publisher(Float32, '/speed', 10)
            self.turn_signal_pub = self.create_publisher(String, '/turn_signals', 10)
            self.trajectory_pub = self.create_publisher(String, '/trajectory_status', 10)
            self.avoidance_pub = self.create_publisher(String, '/avoidance_status', 10)
            self.position_pub = self.create_publisher(String, '/vehicle_position', 10)
            self.waypoint_pub = self.create_publisher(String, '/current_waypoint', 10)
            
            self.get_logger().info('CARLA Data Recorder Node initialized')
    

    
        def publish_odometry(self, vehicle):
            """Publish vehicle odometry"""
            try:
                odometry = Odometry()
                odometry.header.stamp = self.get_clock().now().to_msg()
                odometry.header.frame_id = 'odom'
                
                transform = vehicle.get_transform()
                velocity = vehicle.get_velocity()
                angular_velocity = vehicle.get_angular_velocity()
                
                # Position
                odometry.pose.pose.position.x = transform.location.x
                odometry.pose.pose.position.y = -transform.location.y
                odometry.pose.pose.position.z = transform.location.z
                
                # Orientation (quaternion from Euler)
                quaternion = tf_transformations.quaternion_from_euler(
                    np.deg2rad(transform.rotation.roll),
                    np.deg2rad(transform.rotation.pitch),
                    np.deg2rad(transform.rotation.yaw)
                )
                odometry.pose.pose.orientation.x = quaternion[0]
                odometry.pose.pose.orientation.y = quaternion[1]
                odometry.pose.pose.orientation.z = quaternion[2]
                odometry.pose.pose.orientation.w = quaternion[3]
                
                # Velocity
                odometry.twist.twist.linear.x = velocity.x
                odometry.twist.twist.linear.y = -velocity.y
                odometry.twist.twist.linear.z = velocity.z
                
                # Angular velocity
                odometry.twist.twist.angular.x = angular_velocity.x
                odometry.twist.twist.angular.y = -angular_velocity.y
                odometry.twist.twist.angular.z = angular_velocity.z
                
                self.odom_publisher.publish(odometry)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing odometry: {e}')
        
        def publish_vehicle_control_data(self, throttle, steer, brake, speed):
            """Publish all vehicle control data"""
            try:
                # Individual control values
                throttle_msg = Float32()
                throttle_msg.data = float(throttle)
                self.throttle_pub.publish(throttle_msg)
                
                steer_msg = Float32()
                steer_msg.data = float(steer)
                self.steering_pub.publish(steer_msg)
                
                brake_msg = Float32()
                brake_msg.data = float(brake)
                self.brake_pub.publish(brake_msg)
                
                speed_msg = Float32()
                speed_msg.data = float(speed)
                self.speed_pub.publish(speed_msg)
                
                # Combined control status
                control_status = f"throttle:{throttle:.3f},steer:{steer:.3f},brake:{brake:.3f},speed:{speed:.3f}"
                control_msg = String()
                control_msg.data = control_status
                self.vehicle_control_pub.publish(control_msg)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing vehicle control data: {e}')
        
        def publish_position_data(self, x, y, direction, yaw):
            """Publish vehicle position and orientation data"""
            try:
                position_data = f"x:{x:.3f},y:{y:.3f},direction:{direction},yaw:{yaw:.3f}"
                msg = String()
                msg.data = position_data
                self.position_pub.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing position data: {e}')
        
        def publish_waypoint_data(self, current_waypoints, closest_index):
            """Publish current waypoint information"""
            try:
                if len(current_waypoints) > 0 and closest_index < len(current_waypoints):
                    current_wp = current_waypoints[closest_index]
                    waypoint_data = f"index:{closest_index},x:{current_wp[0]:.3f},y:{current_wp[1]:.3f},total_waypoints:{len(current_waypoints)}"
                else:
                    waypoint_data = f"index:0,x:0.0,y:0.0,total_waypoints:{len(current_waypoints)}"
                
                msg = String()
                msg.data = waypoint_data
                self.waypoint_pub.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing waypoint data: {e}')
        
        def publish_turn_signals(self, left_active, right_active):
            """Publish turn signal status"""
            try:
                if left_active:
                    signal_status = "LEFT"
                elif right_active:
                    signal_status = "RIGHT"
                else:
                    signal_status = "NONE"
                
                msg = String()
                msg.data = signal_status
                self.turn_signal_pub.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing turn signals: {e}')
        
        def publish_trajectory_status(self, status_info):
            """Publish trajectory status information"""
            try:
                msg = String()
                msg.data = status_info
                self.trajectory_pub.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing trajectory status: {e}')
        
        def publish_avoidance_status(self, avoidance_info):
            """Publish obstacle avoidance status"""
            try:
                msg = String()
                msg.data = avoidance_info
                self.avoidance_pub.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing avoidance status: {e}')
        
        def publish_traffic_sign(self, sign_name):
            """Publish detected traffic sign"""
            try:
                msg = String()
                msg.data = sign_name
                self.traffic_sign_pub.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing traffic sign: {e}')
        
        def publish_vehicle_status(self, status_info):
            """Publish vehicle status information"""
            try:
                msg = String()
                msg.data = status_info
                self.vehicle_status_pub.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing vehicle status: {e}')
        
        def publish_cone_detection(self, cone_info):
            """Publish cone detection information"""
            try:
                msg = String()
                msg.data = cone_info
                self.cone_detection_pub.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing cone detection: {e}')
        
        def publish_imu(self, vehicle):
            """Publish IMU data"""
            try:
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'
                
                # Get vehicle acceleration and angular velocity
                acceleration = vehicle.get_acceleration()
                angular_velocity = vehicle.get_angular_velocity()
                transform = vehicle.get_transform()
                
                # Linear acceleration
                imu_msg.linear_acceleration.x = acceleration.x
                imu_msg.linear_acceleration.y = -acceleration.y
                imu_msg.linear_acceleration.z = acceleration.z
                
                # Angular velocity
                imu_msg.angular_velocity.x = angular_velocity.x
                imu_msg.angular_velocity.y = -angular_velocity.y
                imu_msg.angular_velocity.z = angular_velocity.z
                
                # Orientation (quaternion from Euler)
                quaternion = tf_transformations.quaternion_from_euler(
                    np.deg2rad(transform.rotation.roll),
                    np.deg2rad(transform.rotation.pitch),
                    np.deg2rad(transform.rotation.yaw)
                )
                imu_msg.orientation.x = quaternion[0]
                imu_msg.orientation.y = quaternion[1]
                imu_msg.orientation.z = quaternion[2]
                imu_msg.orientation.w = quaternion[3]
                
                self.imu_publisher.publish(imu_msg)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing IMU: {e}')

else:
    # Dummy class if ROS2 is not available
    class CarlaROS2Node:
        def __init__(self):
            print("ROS2 not available - ROS2 functionality disabled")
        

        
        def publish_odometry(self, vehicle):
            pass
        
        def publish_vehicle_control_data(self, throttle, steer, brake, speed):
            pass
        
        def publish_position_data(self, x, y, direction, yaw):
            pass
        
        def publish_waypoint_data(self, current_waypoints, closest_index):
            pass
        
        def publish_turn_signals(self, left_active, right_active):
            pass
        
        def publish_trajectory_status(self, status_info):
            pass
        
        def publish_avoidance_status(self, avoidance_info):
            pass
        
        def publish_traffic_sign(self, sign_name):
            pass
        
        def publish_vehicle_status(self, status_info):
            pass
        
        def publish_cone_detection(self, cone_info):
            pass
        
        def publish_imu(self, vehicle):
            pass
        
        def destroy_node(self):
            pass


class ManualControl:
    """Manual keyboard control for the vehicle"""
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.steer_cache = 0.0
        self.throttle_cache = 0.0
        self.brake_cache = 0.0
        self.reverse = False
        
        # Initialize pygame for keyboard input (minimal setup)
        pygame.init()
        # Create a small hidden window just for keyboard events
        self.screen = pygame.display.set_mode((200, 100))
        pygame.display.set_caption("Manual Control - Press M to toggle")
        
        print("Manual control initialized!")
        print("Press 'M' key to toggle between manual and autonomous mode")
        print("Manual mode controls:")
        print("  W - Throttle")
        print("  S - Brake") 
        print("  A - Steer left")
        print("  D - Steer right")
        print("  SPACE - Hand brake")
        print("  R - Toggle reverse")
        print("  M - Return to autonomous mode")
    
    def parse_keys(self, keys_pressed):
        """Parse keyboard input and return control values"""
        control = carla.VehicleControl()
        
        # Throttle control (W key)
        if keys_pressed[pygame.K_w]:
            self.throttle_cache = min(1.0, self.throttle_cache + 0.05)
        else:
            self.throttle_cache = max(0.0, self.throttle_cache - 0.05)
        
        # Brake control (S key)
        if keys_pressed[pygame.K_s]:
            self.brake_cache = min(1.0, self.brake_cache + 0.1)
        else:
            self.brake_cache = max(0.0, self.brake_cache - 0.05)
        
        # Steering control (A and D keys)
        if keys_pressed[pygame.K_a]:
            self.steer_cache = max(-1.0, self.steer_cache - 0.05)
        elif keys_pressed[pygame.K_d]:
            self.steer_cache = min(1.0, self.steer_cache + 0.05)
        else:
            # Auto-center steering
            if self.steer_cache > 0:
                self.steer_cache = max(0.0, self.steer_cache - 0.03)
            elif self.steer_cache < 0:
                self.steer_cache = min(0.0, self.steer_cache + 0.03)
        
        # Set control values
        control.throttle = self.throttle_cache
        control.brake = self.brake_cache
        control.steer = self.steer_cache
        control.reverse = self.reverse
        
        # Hand brake (SPACE)
        if keys_pressed[pygame.K_SPACE]:
            control.hand_brake = True
        
        return control
    
    def toggle_reverse(self):
        """Toggle reverse gear"""
        self.reverse = not self.reverse
        print(f"Reverse: {'ON' if self.reverse else 'OFF'}")

    def cleanup(self):
        """Clean up pygame resources"""
        pygame.quit()

# Initialize ROS2 (only if available)
if ROS2_AVAILABLE:
    rclpy.init()
    ros2_node = CarlaROS2Node()
else:
    ros2_node = CarlaROS2Node()  # Dummy node

vehicle, camera, camera_data = setup()
s_c = speed_controller.DrivingController() # speed controller

INTERP_LOOKAHEAD_DISTANCE = 1
INTERP_DISTANCE_RES       = 0.01 
closest_index = 0  
closest_distance = 0
stop_event = threading.Event()
control_start_event = threading.Event()

# Add trajectory update communication
trajectory_update_event = threading.Event()
trajectory_lock = threading.Lock()

Global.x_main,Global.y_main= Main.getFinalTrajectory()
# Global.stop_points=[[-48,75.5],[-76,52],[Global.x_main[-1],Global.y_main[-1]]]
Global.stop_points=[[Global.x_main[-1],Global.y_main[-1]]]
Global.control_thread = True  # Initialize the control_thread attribute

######
#vehicle, camera, camera_data = setup()
s_c = speed_controller.DrivingController() # speed controller

INTERP_LOOKAHEAD_DISTANCE = 1
INTERP_DISTANCE_RES       = 0.01 
closest_index = 0  
closest_distance = 0
stop_event = threading.Event()
control_start_event = threading.Event()

# Add trajectory update communication
trajectory_update_event = threading.Event()
trajectory_lock = threading.Lock()

# SIMPLE and WORKING Stop sign variables - based on successful test
stop_sign_active = False
stop_sign_start_time = 0
stop_cooldown_active = False
stop_cooldown_start_time = 0
STOP_DURATION = 3.0  # 3 seconds
STOP_COOLDOWN_DURATION = 5.0  # 5 seconds cooldown after stop

# Mission completion variables
mission_completed = False
final_destination_reached = False
DESTINATION_THRESHOLD = 3.0  # Distance threshold to consider destination reached (meters)

Global.x_main,Global.y_main= Main.getFinalTrajectory()
Global.stop_points=[[Global.x_main[-1],Global.y_main[-1]]]
Global.control_thread = True  # Initialize the control_thread attribute

def set_turn_signal(vehicle, left_signal=False, right_signal=False):
    current_lights = vehicle.get_light_state()
    
    if left_signal:
        current_lights |= carla.VehicleLightState.LeftBlinker
    else:
        current_lights &= ~carla.VehicleLightState.LeftBlinker
        
    if right_signal:
        current_lights |= carla.VehicleLightState.RightBlinker
    else:
        current_lights &= ~carla.VehicleLightState.RightBlinker

    vehicle.set_light_state(carla.VehicleLightState(current_lights))

def stop_threads():
    stop_event.set()

def check_mission_completion(current_x, current_y):
    """Check if vehicle has reached the final destination"""
    global mission_completed, final_destination_reached
    
    # Get the final mission point from Global
    from FinalPathGenerator.util import Global as FinalGlobal
    if hasattr(FinalGlobal, 'mission') and FinalGlobal.mission:
        final_mission = FinalGlobal.mission[-1]  # Last mission point
        
        # Convert mission indices to real world coordinates
        # This is a simplified conversion - you may need to adjust based on your coordinate system
        target_x = final_mission[0] * 10  # Adjust scaling factor as needed
        target_y = final_mission[1] * 10  # Adjust scaling factor as needed
        
        # Calculate distance to final destination
        distance_to_target = math.sqrt((current_x - target_x)**2 + (current_y - target_y)**2)
        
        print(f"🎯 Distance to final destination: {distance_to_target:.2f}m (threshold: {DESTINATION_THRESHOLD}m)")
        print(f"🎯 Current: ({current_x:.1f}, {current_y:.1f}) -> Target: ({target_x:.1f}, {target_y:.1f})")
        
        if distance_to_target <= DESTINATION_THRESHOLD and not final_destination_reached:
            print("🏁🏁🏁 FINAL DESTINATION REACHED! Mission completed!")
            final_destination_reached = True
            mission_completed = True
            Global.breaking = 1.0  # Apply full brake
            return True
            
    return False

def thread_function_1():
    print("1 calisiyor")
    # Use simple YOLOv8 detector with last.pt weights
    detector = torch_det.SimpleYOLOv8Detector('last.pt')
    # cone_detector = cd.ConeDetector()  # CONE DETECTION DISABLED
    
    # Import cone avoidance here to avoid circular import issues
    import detection.cone_avoidance as ca
    obstacle_avoidance = ca.ObstacleAvoidanceSystem()  # Add enhanced obstacle avoidance (walls only)

    # Global variables for simple stop sign system
    global stop_sign_active, stop_sign_start_time, stop_cooldown_active, stop_cooldown_start_time
    global mission_completed, final_destination_reached

    while not stop_event.is_set():
        # 1) Kamera karesini al ve RGB'ye çevir
        frame = camera_data['image'][:, :, :-1]
        # Convert to proper format for processing
        frame = frame.astype(np.uint8)

        # 2) Deteksiyonu yap ve süreyi ölç
        start_time = time.time()
        result = detector.process_frame(frame)
        
        # Handle result returned as a tuple (frame, label)
        if isinstance(result, tuple) and len(result) >= 2:
            frame, levha = result[0], result[1]
            
            # DEBUG: Her durumda levha durumunu yazdır
            print(f"🔍 YOLO SONUCU: levha='{levha}' (tip: {type(levha)})")
            
            # Check for wall detection
            is_wall = False
            if levha and levha != "None":
                levha_clean = str(levha).strip().lower()
                if "duvar" in levha_clean or "wall" in levha_clean:
                    is_wall = True
                    print(f"🧱 WALL DETECTED: '{levha}' -> '{levha_clean}'")
            
            # ADVANCED DUR DETECTION - PRECISE MATCHING (dur vs durak)
            is_dur = False
            if levha and levha != "None":
                levha_clean = str(levha).strip().lower()
                
                # Check for DUR (stop sign) - NOT "durak" (bus stop)
                if levha_clean == "dur":
                    is_dur = True
                    print(f"✅ EXACT DUR MATCH: '{levha}' -> '{levha_clean}'")
                elif "durak" in levha_clean:
                    print(f"🚌 DURAK (bus stop) detected, ignoring: '{levha}'")
                    is_dur = False
                elif "dur" in levha_clean and "durak" not in levha_clean:
                    # Only if "dur" is present but "durak" is not
                    is_dur = True
                    print(f"⚠️ PARTIAL DUR MATCH: '{levha}' -> '{levha_clean}'")
                else:
                    print(f"🔍 Other detection: '{levha}' -> '{levha_clean}'")
            
            # Debug output for all detections
            if levha and levha != "None":
                print(f"🔍 DEBUG: Original='{levha}', Clean='{levha_clean}', is_dur={is_dur}, is_wall={is_wall}")
            
            if is_dur:
                if not stop_sign_active and not stop_cooldown_active:
                    print("🛑🛑🛑 STOP SIGN DETECTED! Starting 3-second stop...")
                    stop_sign_active = True
                    stop_sign_start_time = time.time()
                    Global.breaking = 1.0  # Apply brake
                    print(f"🔥 Brake applied: Global.breaking = {Global.breaking}")
                elif stop_cooldown_active:
                    cooldown_elapsed = time.time() - stop_cooldown_start_time
                    cooldown_remaining = STOP_COOLDOWN_DURATION - cooldown_elapsed
                    print(f"⏱️ STOP COOLDOWN ACTIVE: {cooldown_remaining:.1f}s remaining before accepting new stop signs")
            
            # Check if stop duration is over
            if stop_sign_active:
                elapsed = time.time() - stop_sign_start_time
                if elapsed >= STOP_DURATION:
                    print("✅ Stop duration complete! Starting 5-second cooldown...")
                    stop_sign_active = False
                    stop_cooldown_active = True
                    stop_cooldown_start_time = time.time()
                    Global.breaking = 0.0  # Release brake completely when entering cooldown
                    print(f"🚗 Brake released for cooldown: Global.breaking = {Global.breaking}")
                else:
                    remaining = STOP_DURATION - elapsed
                    print(f"⏱️ STOP ACTIVE: {remaining:.1f}s remaining (brake={Global.breaking})")
                    Global.breaking = 1.0  # Keep brake applied
            
            # Check if cooldown is over
            if stop_cooldown_active:
                cooldown_elapsed = time.time() - stop_cooldown_start_time
                if cooldown_elapsed >= STOP_COOLDOWN_DURATION:
                    print("✅ Cooldown period complete! Ready for new stop signs.")
                    stop_cooldown_active = False
                    Global.breaking = 0.0  # Ensure brake is fully released after cooldown
                    print(f"🚗 Normal driving resumed: Global.breaking = {Global.breaking}")
        else:
            print(f"Unexpected result format: {type(result)}")
            levha = "None"
            # If result isn't a tuple, assume it's just the frame
            if result is not None:
                frame = result
        
        # 2b) Process obstacles (cones and walls)
        
        # Check if wall was detected by YOLO
        wall_detected = False
        if levha != "None" and levha:
            levha_clean = str(levha).strip().lower()
            if "duvar" in levha_clean or "wall" in levha_clean:
                wall_detected = True
                print(f"🧱 WALL detected by YOLO: {levha}")
        
        # Cone detection DISABLED
        yolo_detected_cone = False  # Disable YOLO cone detection
        cone_positions = []  # No cone positions
        
        # Process obstacles - CONE DETECTION DISABLED, only wall detection
        obstacle_changed, obstacle_data = obstacle_avoidance.process_obstacle_detections(
            [], wall_detected=wall_detected)  # Empty cone list
        
        # 2c) If obstacle status changed, regenerate trajectory IMMEDIATELY
        if obstacle_changed:
            obstacle_type = "wall" if wall_detected else "no obstacles"  # Only wall detection active
            print(f"🚨 Obstacle status changed (found {obstacle_type}), regenerating trajectory...")
            # Get current position for avoidance path generation
            if Global.dubaVar:
                try:
                    # Get current position and direction for avoidance path
                    current_x = Global.current_x
                    current_y = Global.current_y
                    direction = Global.direction
                    
                    # Generate avoidance path directly using the obstacle avoidance system
                    x_avoid, y_avoid = obstacle_avoidance.generate_avoidance_path(
                        current_x, current_y, direction)
                    
                    if x_avoid and y_avoid and len(x_avoid) > 0:
                        # IMMEDIATE UPDATE to main trajectory with avoidance path
                        with trajectory_lock:
                            Global.x_main, Global.y_main = x_avoid, y_avoid
                            Global.avoidance_active = True  # Flag to indicate we're in avoidance mode
                            
                        # Set the event to notify control thread of trajectory change
                        trajectory_update_event.set()
                        print(f"✅ New avoidance trajectory generated with {len(Global.x_main)} waypoints")
                        print("🚗 Trajectory updated! Signaling control thread...")
                    else:
                        print("❌ No valid avoidance path returned - EMERGENCY FALLBACK")
                        # Simple emergency avoidance fallback
                        lateral_offset = 3.0
                        x_points = []
                        y_points = []
                        
                        if direction == "K":  # North
                            x_points = [current_x, current_x - lateral_offset, current_x - lateral_offset, current_x]
                            y_points = [current_y, current_y + 2, current_y + 10, current_y + 15]
                        elif direction == "G":  # South
                            x_points = [current_x, current_x + lateral_offset, current_x + lateral_offset, current_x]
                            y_points = [current_y, current_y - 2, current_y - 10, current_y - 15]
                        elif direction == "D":  # East
                            x_points = [current_x, current_x + 2, current_x + 10, current_x + 15]
                            y_points = [current_y, current_y + lateral_offset, current_y + lateral_offset, current_y]
                        elif direction == "B":  # West
                            x_points = [current_x, current_x - 2, current_x - 10, current_x - 15]
                            y_points = [current_y, current_y - lateral_offset, current_y - lateral_offset, current_y]
                        
                        # IMMEDIATE UPDATE to trajectory with emergency avoidance path
                        with trajectory_lock:
                            Global.x_main, Global.y_main = x_points, y_points
                            Global.avoidance_active = True  # Flag to indicate we're in avoidance mode
                        
                        # Signal control thread of the updated trajectory
                        trajectory_update_event.set()
                        print(f"🆘 EMERGENCY avoidance path created with {len(x_points)} points")
                        print("🚗 Trajectory updated! Signaling control thread...")
                except Exception as e:
                    print(f"❌ Error regenerating trajectory: {e}")
                    
                    # As absolute last resort, get the original trajectory if no avoidance path is available
                    with trajectory_lock:
                        if not Global.x_main or len(Global.x_main) == 0:
                            try:
                                Global.x_main, Global.y_main = Main.getFinalTrajectory()
                                print(f"⚠️ Falling back to original trajectory with {len(Global.x_main)} waypoints")
                            except Exception as e:
                                print(f"💥 CRITICAL: Failed to get final trajectory: {e}")
                    
                    # Set the event to notify control thread of trajectory change
                    trajectory_update_event.set()
            else:
                # No obstacle, but check if we need to regenerate normal trajectory
                # Only regenerate if we were previously in avoidance mode
                if getattr(Global, 'avoidance_active', False):
                    try:
                        with trajectory_lock:
                            Global.x_main, Global.y_main = Main.getFinalTrajectory()
                            Global.avoidance_active = False  # Clear avoidance mode
                        # Set the event to notify control thread
                        trajectory_update_event.set()
                        print(f"✅ Returned to normal trajectory with {len(Global.x_main)} waypoints")
                    except Exception as e:
                        print(f"❌ Error regenerating normal trajectory: {e}")
            
        elapsed_time = time.time() - start_time
        print("işlem süresi:", elapsed_time)

        # 3) Ekrana bastır
        # Add timestamp and FPS info on the display
        elapsed_time = time.time() - start_time
        fps = 1.0 / elapsed_time if elapsed_time > 0 else 0
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        # Larger font and adjusted positions for 1080p display
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(frame, f"Detected: {levha}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        
        # Add position and direction info
        cv2.putText(frame, f"Pos: ({Global.current_x:.1f}, {Global.current_y:.1f}) Dir: {Global.direction}", 
                   (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
        
        # Mission completion visual feedback
        if mission_completed:
            # Green background for mission completed
            cv2.rectangle(frame, (10, 180), (900, 240), (0, 255, 0), -1)
            cv2.rectangle(frame, (10, 180), (900, 240), (255, 255, 255), 3)  # White border
            cv2.putText(frame, f"🏁 MISSION COMPLETED - VEHICLE STOPPED 🏁", 
                       (20, 220), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 3)
        # SIMPLE Stop sign visual feedback
        elif stop_sign_active:
            elapsed_time = time.time() - stop_sign_start_time
            remaining_time = STOP_DURATION - elapsed_time
            if remaining_time > 0:
                # Kırmızı arka plan ile dur uyarısı - daha büyük ve belirgin
                cv2.rectangle(frame, (10, 180), (800, 240), (0, 0, 255), -1)
                cv2.rectangle(frame, (10, 180), (800, 240), (255, 255, 255), 3)  # Beyaz çerçeve
                cv2.putText(frame, f"🛑 DUR LEVHASI AKTIF - BEKLEME: {remaining_time:.1f}s", 
                           (20, 220), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
                
                # İkinci satırda tespit edilen levha ismini göster
                cv2.putText(frame, f"Tespit edilen: {levha}", 
                           (20, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        # Cooldown visual feedback
        elif stop_cooldown_active:
            cooldown_elapsed = time.time() - stop_cooldown_start_time
            cooldown_remaining = STOP_COOLDOWN_DURATION - cooldown_elapsed
            if cooldown_remaining > 0:
                # Sarı arka plan ile cooldown uyarısı
                cv2.rectangle(frame, (10, 250), (800, 310), (0, 180, 255), -1)  # Turuncu arka plan (position adjusted)
                cv2.rectangle(frame, (10, 250), (800, 310), (255, 255, 255), 2)  # Beyaz çerçeve
                cv2.putText(frame, f"⏱️ COOLDOWN - YENİ DUR LEVHASI İÇİN BEKLEME: {cooldown_remaining:.1f}s", 
                           (20, 290), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                
                # İkinci satırda ek bilgi
                cv2.putText(frame, "Araç tekrar harekete geçebilir", 
                           (20, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        
        # Add obstacle detection info with improved visibility - CONE DETECTION DISABLED
        if wall_detected:  # Only show wall detection
            cone_y_position = 350 if mission_completed or stop_sign_active or stop_cooldown_active else 250
            
            cv2.putText(frame, f"WALL DETECTED: Obstacle ahead", 
                       (20, cone_y_position), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            
            # Draw a prominent warning for obstacle avoidance
            if Global.dubaVar:
                # Create a filled rectangle for better visibility - adjust position based on stop sign or cooldown
                warning_y_start = 380 if mission_completed or stop_sign_active or stop_cooldown_active else 280
                warning_y_end = warning_y_start + 50
                cv2.rectangle(frame, (10, warning_y_start), (600, warning_y_end), (0, 0, 255), -1)
                
                cv2.putText(frame, f"⚠️ WALL AVOIDANCE ACTIVE ⚠️", 
                           (20, warning_y_start + 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                
                # Show obstacle details
                obstacle_details = f"Obstacle data: {Global.dubaEngel}"
                cv2.putText(frame, obstacle_details, (20, warning_y_end + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Timestamp'i en alta taşı
        timestamp_y = 550 if mission_completed or stop_sign_active or stop_cooldown_active else 430
        cv2.putText(frame, timestamp, (20, timestamp_y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        
        cv2.imshow('Full HD RGB Camera (1920x1080)', frame)
        print("levha:", levha)
        
        # Publish ROS2 data for rosbag recording
        try:
            # ROS2 data publishing
            
            # Publish traffic sign detection
            if levha and levha != "None":
                ros2_node.publish_traffic_sign(levha)
            
            # Publish cone detection information - DISABLED
            # Cone detection is disabled, only publish if wall is detected
            if wall_detected:
                cone_info = f"wall_detected:true,obstacle_type:wall"
                ros2_node.publish_cone_detection(cone_info)
            
            # Publish avoidance status - cone detection disabled
            if Global.dubaVar or getattr(Global, 'avoidance_active', False):
                avoidance_info = f"avoidance_active:true,obstacle_type:{'wall' if wall_detected else 'unknown'},trajectory_points:{len(Global.x_main) if hasattr(Global, 'x_main') else 0}"
                ros2_node.publish_avoidance_status(avoidance_info)
            else:
                ros2_node.publish_avoidance_status("avoidance_active:false")
                
            # Spin ROS2 node to process callbacks
            if ROS2_AVAILABLE:
                rclpy.spin_once(ros2_node, timeout_sec=0.001)
            
        except Exception as e:
            print(f"Error publishing ROS2 data in thread 1: {e}")
        
        # Add debugging info for detection - cone detection disabled
        if levha != "None":
            detection_type = ""
            if wall_detected:
                detection_type = " (WALL DETECTED)"
            # Cone detection is disabled
            print(f"DETECTION: Sign '{levha}'{detection_type} detected, Current position: ({Global.current_x}, {Global.current_y}), Direction: {Global.direction}")

        # Store previous trajectory for comparison
        with trajectory_lock:
            prev_x = Global.x_main.copy() if hasattr(Global, 'x_main') and Global.x_main else []
            prev_y = Global.y_main.copy() if hasattr(Global, 'y_main') and Global.y_main else []

        # 4) Levha bilgisiyle global state güncelle
        Levha.levha(levha,
                    Global.direction,
                    Global.current_x,
                    Global.current_y)

        # Check if trajectory changed (from sign detection or obstacle changes) and signal thread 2
        with trajectory_lock:
            if (obstacle_changed or 
                not prev_x or not prev_y or 
                len(Global.x_main) != len(prev_x) or 
                len(Global.y_main) != len(prev_y) or
                not np.array_equal(Global.x_main, prev_x) or 
                not np.array_equal(Global.y_main, prev_y)):
                
                print("Trajectory updated! Signaling control thread...")
                trajectory_update_event.set()

        # 5) Signal thread2 to start working
        if Global.control_thread:
            control_start_event.set()
            Global.control_thread = False

        # 6) 'q' ile çıkış
        key = cv2.waitKey(1)
        if key == ord('q'):
            print("Q tuşuna basıldı, araç ve kamera siliniyor...")
            stop_threads()
            break

def thread_function_2():
    print("2 calisiyor - waiting for signal")
    
    # Global variables for simple stop sign system
    global stop_sign_active, stop_sign_start_time, stop_cooldown_active, stop_cooldown_start_time
    global mission_completed, final_destination_reached
    
    control_start_event.wait()  # Wait until signaled
    print("2 starting control loop")
    
    # Initialize waypoints
    with trajectory_lock:
        current_waypoints = np.array(list(zip(Global.x_main, Global.y_main)))
    
    # Print the initial waypoints
    print(f"Initial control waypoints: {len(current_waypoints)} points")
    if len(current_waypoints) > 0:
        print(f"First point: {current_waypoints[0]}")
        if len(current_waypoints) > 1:
            print(f"Last point: {current_waypoints[-1]}")
    
    while not stop_event.is_set():
        # Check for trajectory updates - HIGHEST PRIORITY
        if trajectory_update_event.is_set():
            print("🔄 Trajectory update detected - updating control waypoints...")
            with trajectory_lock:
                # Ensure x_main and y_main are valid
                if (hasattr(Global, 'x_main') and hasattr(Global, 'y_main') and 
                    Global.x_main is not None and Global.y_main is not None and
                    len(Global.x_main) > 0 and len(Global.y_main) > 0):
                    
                    # IMMEDIATELY UPDATE the waypoints from the global trajectory
                    current_waypoints = np.array(list(zip(Global.x_main, Global.y_main)))
                    print(f"✅ Updated waypoints: {len(current_waypoints)} points")
                    
                    # Print the first and last few points of the new trajectory
                    if len(current_waypoints) > 0:
                        print(f"First point: {current_waypoints[0]}")
                        if len(current_waypoints) > 1:
                            print(f"Last point: {current_waypoints[-1]}")
                            
                    # Check if we're in avoidance mode
                    if getattr(Global, 'avoidance_active', False):
                        print("⚠️ AVOIDANCE MODE ACTIVE - following avoidance waypoints")
                else:
                    print("❌ Warning: Invalid x_main or y_main in Global, keeping previous waypoints")
            
            # Clear the event after processing
            trajectory_update_event.clear()
        
        # Ensure we have valid waypoints before proceeding
        if len(current_waypoints) == 0:
            print("No valid waypoints, trying to regenerate")
            try:
                with trajectory_lock:
                    Global.x_main, Global.y_main = Main.getFinalTrajectory()
                    current_waypoints = np.array(list(zip(Global.x_main, Global.y_main)))
                print(f"Regenerated waypoints: {len(current_waypoints)} points")
            except Exception as e:
                print(f"❌ Error regenerating waypoints: {e}")
                time.sleep(0.1)  # Avoid tight loop
                continue
        
        # Use current waypoints for control
        try:
            wp_interp, wp_distance, wp_interp, wp_interp_hash = WaypointInterpolation.interpolate_waypoints(current_waypoints, INTERP_DISTANCE_RES)
        except Exception as e:
            print(f"❌ Error interpolating waypoints: {e}")
            time.sleep(0.1)  # Avoid tight loop
            continue
        
        location = vehicle.get_location()
        
        # Update global position variables
        Global.current_x = location.x
        Global.current_y = location.y
        
        velocity = vehicle.get_velocity()
        transform = vehicle.get_transform()
        Global.yaw = transform.rotation.yaw  # Make sure yaw is updated
        
        # Calculate current speed
        current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6  # Convert to km/h
        
        # Update vehicle direction based on yaw
        from Controller.yon import get_direction
        get_direction()  # This updates Global.direction
        
        # Check if mission is completed (reached final destination)
        if not mission_completed:
            destination_reached = check_mission_completion(Global.current_x, Global.current_y)
        
        try:
            # Get vehicle controls
            throttle = s_c.update(vehicle)
            steer = lc.update(vehicle, current_waypoints, wp_distance, wp_interp, wp_interp_hash, INTERP_LOOKAHEAD_DISTANCE, closest_index, closest_distance)
            
            # MISSION COMPLETION CHECK - highest priority
            if mission_completed:
                # MISSION COMPLETED - STOP THE VEHICLE
                throttle = 0.0
                Global.breaking = 1.0
                print(f"🏁 MISSION COMPLETED - VEHICLE STOPPED: brake={Global.breaking}, throttle={throttle}")
            # SIMPLE Stop sign control logic
            elif stop_sign_active:
                # FORCE STOP - override everything, use Global.breaking
                throttle = 0.0
                Global.breaking = 1.0
                print(f"🛑 FORCING STOP: brake={Global.breaking}, throttle={throttle}")
            elif stop_cooldown_active:
                # During cooldown, allow normal driving but log the status
                print(f"⏱️ In cooldown: brake={Global.breaking}, throttle={throttle}")
            else:
                # Normal driving - respect Global.breaking setting
                if Global.breaking > 0:
                    print(f"🚗 Normal w/ brake: brake={Global.breaking}, throttle={throttle}")
                else:
                    print(f"🚗 Normal driving: brake={Global.breaking}, throttle={throttle}")
            
            # Reduce speed when in avoidance mode for safety
            if getattr(Global, 'avoidance_active', False):
                print(f"🐢 Reduced speed for avoidance (throttle: {throttle:.2f} -> {throttle * 0.7:.2f})")
                throttle *= 0.7  # Reduce throttle by 30% during avoidance
            
            # Apply vehicle controls
            control_command = carla.VehicleControl(throttle=throttle, steer=steer, brake=Global.breaking)
            vehicle.apply_control(control_command)
            
            # Set turn signals based on steering angle
            left_signal = steer < -0.1
            right_signal = steer > 0.1
            
            if right_signal:
                set_turn_signal(vehicle, left_signal=False, right_signal=True)
            elif left_signal:
                set_turn_signal(vehicle, left_signal=True, right_signal=False)
            else:
                set_turn_signal(vehicle, left_signal=False, right_signal=False)
                
            # Publish ROS2 data for rosbag recording
            try:
                # Publish vehicle odometry
                ros2_node.publish_odometry(vehicle)
                
                # Publish IMU data
                ros2_node.publish_imu(vehicle)
                
                # Publish vehicle control data
                ros2_node.publish_vehicle_control_data(throttle, steer, Global.breaking, current_speed)
                
                # Publish position data
                ros2_node.publish_position_data(Global.current_x, Global.current_y, Global.direction, Global.yaw)
                
                # Publish waypoint data
                ros2_node.publish_waypoint_data(current_waypoints, closest_index)
                
                # Publish turn signals
                ros2_node.publish_turn_signals(left_signal, right_signal)
                
                # Publish trajectory status
                trajectory_status = f"total_waypoints:{len(current_waypoints)},current_index:{closest_index},avoidance_mode:{getattr(Global, 'avoidance_active', False)}"
                ros2_node.publish_trajectory_status(trajectory_status)
                
                # Publish vehicle status
                vehicle_status = f"speed:{current_speed:.2f},throttle:{throttle:.3f},brake:{Global.breaking:.3f},steer:{steer:.3f},stop_sign_active:{stop_sign_active},cooldown_active:{stop_cooldown_active},mission_completed:{mission_completed}"
                ros2_node.publish_vehicle_status(vehicle_status)
                
                # Spin ROS2 node to process callbacks
                if ROS2_AVAILABLE:
                    rclpy.spin_once(ros2_node, timeout_sec=0.001)
                
            except Exception as e:
                print(f"Error publishing ROS2 data in thread 2: {e}")
                
        except Exception as e:
            print(f"❌ Error in control loop: {e}")
            # Apply safe defaults
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
            set_turn_signal(vehicle, left_signal=False, right_signal=False)

        State.set_state()
        Referans.set_referance()
        
        # Only print stop points in non-avoidance mode to reduce console clutter
        if not getattr(Global, 'avoidance_active', False):
            print(Global.stop_points)

# Thread'leri tanımlama
thread1 = threading.Thread(target=thread_function_1)
thread2 = threading.Thread(target=thread_function_2)

# Thread'leri başlatma
thread1.start()
thread2.start()

# Kullanıcının bir komutla thread'leri durdurmasını sağlamak için bekleme
input("Thread'leri durdurmak için Enter'a basın...\n")
stop_threads()

# Thread'lerin bitmesini bekleme (Bu durumda sonsuza kadar beklerler çünkü while True kullanıldı)
thread1.join()
thread2.join()

# Clean up resources after threads have finished
print("Cleaning up resources...")
if 'camera' in globals() and camera.is_listening:
    camera.stop()
if 'camera' in globals() and camera.is_alive:
    camera.destroy()
if 'vehicle' in globals() and vehicle.is_alive:
    vehicle.destroy()
cv2.destroyAllWindows()

# Clean up ROS2 (only if available)
if ROS2_AVAILABLE:
    try:
        ros2_node.destroy_node()
        rclpy.shutdown()
        print("ROS2 node cleaned up successfully")
    except Exception as e:
        print(f"Error cleaning up ROS2: {e}")
else:
    print("ROS2 was not available - no cleanup needed")

print("Cleanup complete.")
