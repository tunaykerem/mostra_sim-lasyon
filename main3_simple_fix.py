#!/usr/bin/env python3
"""
Simplified main3.py with WORKING stop sign detection using YOLOv8
This is a minimal fix that demonstrates the stop-for-3-seconds functionality
"""

import cv2
import time
import threading
import numpy as np
from setup import setup
from Util import Global
from Controller import speed_controller
from Controller import lateral_controller as lc
from Util import Referans, State
import detection.torch_detector as torch_det  # Use YOLOv8 detector
import carla

# Initialize vehicle and camera
vehicle, camera, camera_data = setup()
s_c = speed_controller.DrivingController()

# Stop sign variables - SIMPLE AND GLOBAL
stop_sign_active = False
stop_sign_start_time = 0
STOP_DURATION = 3.0  # 3 seconds

# Park sign variables - NEW PARKING SYSTEM
park_sign_detected = False
park_active = False
park_completed = False
park_start_time = 0
park_cooldown_time = 0
PARK_APPROACH_DURATION = 2.0  # 2 seconds to approach parking spot
PARK_COOLDOWN_DURATION = 10.0  # 10 seconds cooldown before allowing new park detection

# Thread synchronization
stop_event = threading.Event()

def thread_function_1():
    """Detection thread - using YOLOv8"""
    global stop_sign_active, stop_sign_start_time, park_sign_detected, park_active, park_start_time, park_completed, park_cooldown_time
    
    # Use YOLOv8 detector with the weights from your original setup
    detector = torch_det.SimpleYOLOv8Detector('last.pt')
    print("🔍 YOLOv8 detector initialized")
    
    while not stop_event.is_set():
        try:
            # Get camera frame
            frame = camera_data['image'][:, :, :-1]  # Convert RGBA to RGB
            frame = frame.astype(np.uint8)
            
            # Process with YOLOv8
            result = detector.process_frame(frame)
            
            if isinstance(result, tuple) and len(result) >= 2:
                frame, levha = result[0], result[1]
                
                print(f"🔍 YOLOv8 Detection: '{levha}'")
                
                # SIMPLE DUR DETECTION - PRECISE MATCHING
                is_dur = False
                is_park = False
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
                    
                    # Check for PARK (parking sign) - SEPARATE CHECK
                    # Only allow parking if not already completed or if cooldown has expired
                    current_time = time.time()
                    if park_completed and (current_time - park_cooldown_time) > PARK_COOLDOWN_DURATION:
                        # Reset parking state after cooldown
                        park_completed = False
                        park_sign_detected = False
                        park_active = False
                        print(f"🔄 Parking cooldown expired, ready for new parking detection")
                    
                    if "park" in levha_clean and not park_sign_detected and not park_active and not park_completed:
                        is_park = True
                        print(f"🅿️ PARK SIGN DETECTED: '{levha}' -> '{levha_clean}'")
                        
                        # IMMEDIATE PARK SIGN HANDLING - ONLY ONCE
                        print("🅿️🅿️🅿️ PARK SIGN DETECTED! Starting parking approach...")
                        park_sign_detected = True
                        park_active = True
                        park_start_time = time.time()
                        Global.breaking = 0.0  # Make sure brake is released for parking maneuver
                        print(f"🚗 Starting parking maneuver")
                    elif "park" in levha_clean and park_completed:
                        print(f"🅿️ Park sign detected but parking already completed - ignoring")
                        is_park = False
                    
                    # Default message if neither dur nor park
                    if not is_dur and not is_park:
                        print(f"🔍 Other detection: '{levha}' -> '{levha_clean}'")
                     # Debug output for all detections
            if levha and levha != "None":
                print(f"🔍 DEBUG: Original='{levha}', Clean='{levha_clean}', is_dur={is_dur}, is_park={is_park}")
            
            # MANUAL TEST MODE - Force park detection after 10 seconds (for testing)
            current_time = time.time()
            if not hasattr(thread_function_1, 'start_time'):
                thread_function_1.start_time = current_time
            
            # TEST: Force park detection after 10 seconds for testing
            if (current_time - thread_function_1.start_time) > 10 and not park_sign_detected and not park_active and not park_completed:
                print("🧪 TEST MODE: Forcing park detection for testing...")
                is_park = True
                levha_clean = "park"
                park_sign_detected = True
                park_active = True
                park_start_time = time.time()
                Global.breaking = 0.0
                print(f"🅿️🅿️🅿️ TEST PARK TRIGGERED! Starting parking approach...")
            
            if is_dur and not park_sign_detected and not park_active:
                    if not stop_sign_active:
                        print("🛑🛑🛑 STOP SIGN DETECTED! Starting 3-second stop...")
                        stop_sign_active = True
                        stop_sign_start_time = time.time()
                        Global.breaking = 1.0  # Apply brake
                        print(f"🔥 Brake applied: Global.breaking = {Global.breaking}")
            elif is_dur and (park_sign_detected or park_active):
                print("🛑 Stop sign detected but parking in progress - ignoring stop sign")
            
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
            
            # Check if parking approach is complete
            if park_active:
                elapsed = time.time() - park_start_time
                if elapsed >= PARK_APPROACH_DURATION:
                    print("🅿️ Parking approach complete! FINAL STOP for parking...")
                    park_active = False  # Stop the approach phase
                    Global.breaking = 1.0  # Apply final brake for parking
                    print(f"🅿️ PARKED: Final brake applied, vehicle stopped at parking spot")
                else:
                    remaining = PARK_APPROACH_DURATION - elapsed
                    print(f"🅿️ PARKING APPROACH: {remaining:.1f}s remaining - moving to parking spot")
                    Global.breaking = 0.0  # Keep brake released during parking approach
                    Global.breaking = 0.0  # Keep brake released during parking maneuver
            
            # Display the frame
            cv2.imshow('YOLOv8 Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            time.sleep(0.1)  # Small delay
            
        except Exception as e:
            print(f"❌ Error in detection thread: {e}")
            time.sleep(0.1)

def thread_function_2():
    """Control thread - simplified with brake/throttle override"""
    global stop_sign_active, park_active, park_sign_detected, park_completed, park_cooldown_time
    
    while not stop_event.is_set():
        try:
            # Get current position
            transform = vehicle.get_transform()
            Global.current_x = transform.location.x
            Global.current_y = transform.location.y
            
            # Apply control
            control = carla.VehicleControl()
            
            if park_active:
                # PARKING MANEUVER - turn left and approach parking spot (HIGHEST PRIORITY)
                current_time = time.time()
                parking_duration = current_time - park_start_time
                
                if parking_duration < PARK_APPROACH_DURATION:
                    # Still approaching parking spot
                    control.throttle = 0.3  # Normal speed for parking approach
                    control.brake = 0.0     # No brake during parking maneuver
                    control.steer = -0.7    # Turn left (negative steering) - stronger turn
                    print(f"🅿️ PARKING MANEUVER: turning left and approaching parking spot (throttle={control.throttle}, steer={control.steer}, duration={parking_duration:.1f}s)")
                else:
                    # Parking approach complete - stop and park
                    control.throttle = 0.0
                    control.brake = 1.0
                    control.steer = 0.0
                    park_active = False  # End parking maneuver
                    park_completed = True  # Mark parking as completed
                    park_cooldown_time = current_time  # Start cooldown timer
                    Global.breaking = 1.0  # Set global brake to maintain stop
                    print(f"🅿️ PARKING COMPLETED: Vehicle stopped at parking spot, starting cooldown")
            elif park_completed and not park_active:
                # PARKED STATE - stay stopped (SECOND HIGHEST PRIORITY)
                control.throttle = 0.0
                control.brake = 1.0  # Full brake to stay parked
                control.steer = 0.0
                print(f"🅿️ PARKED: Vehicle maintaining position at parking spot")
            elif park_sign_detected and not park_active and not park_completed:
                # LEGACY STATE - this should not happen with new logic
                control.throttle = 0.0
                control.brake = 1.0  # Full brake to stay parked
                control.steer = 0.0
                print(f"🅿️ LEGACY PARKED: Vehicle stopped at parking spot")
            elif stop_sign_active:
                # FORCE STOP - override everything, use Global.breaking
                control.throttle = 0.0
                control.brake = Global.breaking  # Use the Global brake value
                control.steer = 0.0
                print(f"🛑 FORCING STOP: brake={control.brake}, throttle={control.throttle}, Global.breaking={Global.breaking}")
            else:
                # Normal driving (simplified)
                control.throttle = 0.3  # Basic forward movement
                control.brake = Global.breaking  # Respect Global brake setting
                control.steer = 0.0
                if Global.breaking > 0:
                    print(f"🚗 Normal w/ brake: brake={control.brake}, throttle={control.throttle}")
                else:
                    print(f"🚗 Normal driving: brake={control.brake}, throttle={control.throttle}")
            
            # Apply control to vehicle
            vehicle.apply_control(control)
            
            time.sleep(0.05)  # 20 FPS control loop
            
        except Exception as e:
            print(f"❌ Error in control thread: {e}")
            time.sleep(0.1)

def main():
    """Main function"""
    print("🚀 Starting simplified stop sign detection system...")
    
    # Start threads
    detection_thread = threading.Thread(target=thread_function_1)
    control_thread = threading.Thread(target=thread_function_2)
    
    detection_thread.start()
    control_thread.start()
    
    try:
        # Keep main thread alive
        while True:
            time.sleep(1)
            if stop_sign_active:
                print("🛑 STOP SIGN ACTIVE - Vehicle should be stopped")
            elif park_active:
                print("🅿️ PARKING MANEUVER ACTIVE - Vehicle approaching parking spot")
            elif park_sign_detected:
                print("🅿️ PARKED - Vehicle stopped at parking spot")
            else:
                print("🚗 Normal operation")
                
    except KeyboardInterrupt:
        print("🛑 Stopping system...")
        stop_event.set()
        
        detection_thread.join()
        control_thread.join()
        
        # Cleanup
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        
        print("✅ System stopped cleanly")

if __name__ == "__main__":
    main()
