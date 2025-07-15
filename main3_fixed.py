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

######
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

# SIMPLE and WORKING Stop sign variables - based on successful test
stop_sign_active = False
stop_sign_start_time = 0
STOP_DURATION = 3.0  # 3 seconds

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

def thread_function_1():
    print("1 calisiyor")
    # Use simple YOLOv8 detector with last.pt weights
    detector = torch_det.SimpleYOLOv8Detector('last.pt')
    cone_detector = cd.ConeDetector()  # Add cone detector
    
    # Import cone avoidance here to avoid circular import issues
    import detection.cone_avoidance as ca
    cone_avoidance = ca.ConeAvoidanceSystem()  # Add enhanced cone avoidance

    # Global variables for simple stop sign system
    global stop_sign_active, stop_sign_start_time

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
            
            # SIMPLE and WORKING Stop Sign Detection
            is_dur = False
            if levha and levha != "None":
                levha_clean = str(levha).strip().lower()
                if levha_clean == "dur" or "dur" in levha_clean:
                    is_dur = True
            
            if is_dur:
                if not stop_sign_active:
                    print("🛑🛑🛑 STOP SIGN DETECTED! Starting 3-second stop...")
                    stop_sign_active = True
                    stop_sign_start_time = time.time()
                    Global.breaking = 1.0  # Apply brake
                    print(f"🔥 Brake applied: Global.breaking = {Global.breaking}")
            
            # Check if stop duration is over
            if stop_sign_active:
                elapsed = time.time() - stop_sign_start_time
                if elapsed >= STOP_DURATION:
                    print("✅ Stop duration complete! Resuming...")
                    stop_sign_active = False
                    Global.breaking = 0.0  # Release brake
                    print(f"🚗 Brake released: Global.breaking = {Global.breaking}")
                else:
                    remaining = STOP_DURATION - elapsed
                    print(f"⏱️ STOP ACTIVE: {remaining:.1f}s remaining (brake={Global.breaking})")
                    Global.breaking = 1.0  # Keep brake applied
        else:
            print(f"Unexpected result format: {type(result)}")
            levha = "None"
            # If result isn't a tuple, assume it's just the frame
            if result is not None:
                frame = result
        
        # 2b) Cone detection (only run if YOLO didn't already detect a cone)
        yolo_detected_cone = (levha.lower() in ["koni", "cone"]) if levha != "None" else False
        
        cone_positions = []
        if not yolo_detected_cone:
            # Use traditional cone detector to detect cones
            cone_positions, frame = cone_detector.detect_cones(frame)
            
            # Process cone positions with enhanced avoidance system
            obstacle_changed, obstacle_data = cone_avoidance.process_cone_detections(cone_positions)
        else:
            # YOLO detected a cone, so process it for avoidance
            # Assume cone is directly in front (simple implementation)
            cone_positions = [[0, 3.0]]  # Assume cone is 3m ahead, centered
            obstacle_changed, obstacle_data = cone_avoidance.process_cone_detections(cone_positions)
            print(f"🔴 YOLO detected cone: {levha} - processing for avoidance")
        
        # 2c) If obstacle status changed, regenerate trajectory IMMEDIATELY
        if obstacle_changed:
            print(f"🚨 Obstacle status changed (found {len(cone_positions)} cones), regenerating trajectory...")
            # Get current position for avoidance path generation
            if Global.dubaVar:
                try:
                    # Get current position and direction for avoidance path
                    current_x = Global.current_x
                    current_y = Global.current_y
                    direction = Global.direction
                    
                    # Generate avoidance path directly using the cone avoidance system
                    x_avoid, y_avoid = cone_avoidance.generate_avoidance_path(
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
        # Add timestamp and FPS on the image
        elapsed_time = time.time() - start_time
        fps = 1.0 / elapsed_time if elapsed_time > 0 else 0
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        # Larger font and adjusted positions for 1080p display
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(frame, f"Detected: {levha}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        
        # Add position and direction info
        cv2.putText(frame, f"Pos: ({Global.current_x:.1f}, {Global.current_y:.1f}) Dir: {Global.direction}", 
                   (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
        
        # SIMPLE Stop sign visual feedback
        if stop_sign_active:
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
        
        # Add cone detection info with improved visibility
        if len(cone_positions) > 0:
            closest_cone = min(cone_positions, key=lambda pos: math.sqrt(pos[0]**2 + pos[1]**2))
            # Use red text with yellow background for high visibility - adjust position based on stop sign
            cone_y_position = 280 if stop_sign_active else 250
            cv2.putText(frame, f"CONE DETECTED: {closest_cone[1]:.1f}m ahead, {closest_cone[0]:.1f}m offset", 
                       (20, cone_y_position), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            
            # Draw a prominent warning for obstacle avoidance
            if Global.dubaVar:
                # Create a filled rectangle for better visibility - adjust position based on stop sign
                warning_y_start = 310 if stop_sign_active else 280
                warning_y_end = warning_y_start + 50
                cv2.rectangle(frame, (10, warning_y_start), (600, warning_y_end), (0, 0, 255), -1)
                cv2.putText(frame, "⚠️ OBSTACLE AVOIDANCE ACTIVE ⚠️", 
                           (20, warning_y_start + 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                
                # Show obstacle details
                obstacle_details = f"Obstacle data: {Global.dubaEngel}"
                cv2.putText(frame, obstacle_details, (20, warning_y_end + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                # Show current avoidance path if available
                if hasattr(Global, 'avoidance_path') and Global.avoidance_path:
                    path_info = f"Avoidance path: {len(Global.avoidance_path)} points"
                    cv2.putText(frame, path_info, (20, warning_y_end + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        # Timestamp'i en alta taşı
        timestamp_y = 480 if stop_sign_active else 430
        cv2.putText(frame, timestamp, (20, timestamp_y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        
        cv2.imshow('Full HD RGB Camera (1920x1080)', frame)
        print("levha:", levha)
        
        # Add debugging info for detection
        if levha != "None":
            print(f"DETECTION: Sign '{levha}' detected, Current position: ({Global.current_x}, {Global.current_y}), Direction: {Global.direction}")

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
    global stop_sign_active, stop_sign_start_time
    
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
        
        # Update vehicle direction based on yaw
        from Controller.yon import get_direction
        get_direction()  # This updates Global.direction
        
        try:
            # Get vehicle controls
            throttle = s_c.update(vehicle)
            steer = lc.update(vehicle, current_waypoints, wp_distance, wp_interp, wp_interp_hash, INTERP_LOOKAHEAD_DISTANCE, closest_index, closest_distance)
            
            # SIMPLE Stop sign control logic
            if stop_sign_active:
                # FORCE STOP - override everything, use Global.breaking
                throttle = 0.0
                Global.breaking = 1.0
                print(f"🛑 FORCING STOP: brake={Global.breaking}, throttle={throttle}")
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
            if steer > 0.1:
                set_turn_signal(vehicle, left_signal=False, right_signal=True)
            elif steer < -0.1:
                set_turn_signal(vehicle, left_signal=True, right_signal=False)
            else:
                set_turn_signal(vehicle, left_signal=False, right_signal=False)
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
print("Cleanup complete.")
