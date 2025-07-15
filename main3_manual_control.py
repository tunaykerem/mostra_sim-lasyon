#!/usr/bin/env python3
"""
WASD Manual Control System for CARLA Vehicle
Based on main3.py but with keyboard control instead of autonomous driving
"""

import cv2
import time
import threading
import numpy as np
import pygame
from setup import setup
from Util import Global
from Controller import speed_controller
from Controller import lateral_controller as lc
from Util import Referans, State
import detection.torch_detector as torch_det
import carla

# Initialize vehicle and camera
vehicle, camera, camera_data = setup()
s_c = speed_controller.DrivingController()

# Manual control variables
manual_control_active = True
current_throttle = 0.0
current_steer = 0.0
current_brake = 0.0

# Stop sign variables (keeping from original)
stop_sign_active = False
stop_sign_start_time = 0
STOP_DURATION = 3.0

# Thread synchronization
stop_event = threading.Event()

# Initialize pygame for keyboard input
pygame.init()
pygame.display.set_mode((1, 1))  # Minimal display for key events

def set_turn_signal(vehicle, left_signal=False, right_signal=False):
    """Set vehicle turn signals"""
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

def manual_control_thread():
    """Thread for handling keyboard input and manual control"""
    global manual_control_active, current_throttle, current_steer, current_brake
    global stop_sign_active, stop_sign_start_time
    
    print("🎮 Manual Control Thread Started!")
    print("🎮 WASD Controls:")
    print("   W - Forward (throttle)")
    print("   S - Backward (reverse/brake)")
    print("   A - Turn Left")
    print("   D - Turn Right")
    print("   SPACE - Brake")
    print("   M - Toggle Manual/Auto mode")
    print("   Q - Quit")
    
    clock = pygame.time.Clock()
    
    while not stop_event.is_set():
        try:
            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    stop_threads()
                    break
            
            # Get current key states
            keys = pygame.key.get_pressed()
            
            # Reset controls
            throttle = 0.0
            steer = 0.0
            brake = 0.0
            
            # Manual control inputs
            if manual_control_active:
                # Forward/Backward control
                if keys[pygame.K_w]:  # W key - forward
                    throttle = 0.6
                    print("🚗 Forward (W)")
                elif keys[pygame.K_s]:  # S key - reverse/brake
                    if vehicle.get_velocity().length() > 0.1:  # If moving forward, brake first
                        brake = 1.0
                        print("🛑 Braking (S)")
                    else:  # If stopped, reverse
                        throttle = -0.4
                        print("🔄 Reverse (S)")
                
                # Steering control
                if keys[pygame.K_a]:  # A key - turn left
                    steer = -0.8
                    print("⬅️ Turn Left (A)")
                elif keys[pygame.K_d]:  # D key - turn right
                    steer = 0.8
                    print("➡️ Turn Right (D)")
                
                # Brake control
                if keys[pygame.K_SPACE]:  # Space - brake
                    brake = 1.0
                    throttle = 0.0
                    print("🛑 Emergency Brake (SPACE)")
                
                # Toggle manual/auto mode
                if keys[pygame.K_m]:
                    manual_control_active = False
                    print("🤖 Switching to AUTO mode")
                    time.sleep(0.3)  # Prevent rapid toggling
            else:
                # Auto mode - check if user wants to switch back
                if keys[pygame.K_m]:
                    manual_control_active = True
                    print("🎮 Switching to MANUAL mode")
                    time.sleep(0.3)  # Prevent rapid toggling
            
            # Quit
            if keys[pygame.K_q]:
                print("🛑 Q pressed - stopping system...")
                stop_threads()
                break
            
            # Update global control values
            current_throttle = throttle
            current_steer = steer
            current_brake = brake
            
            # Override for stop sign (even in manual mode for safety)
            if stop_sign_active:
                current_throttle = 0.0
                current_brake = 1.0
                print(f"🛑 STOP SIGN OVERRIDE: forcing stop")
            
            clock.tick(30)  # 30 FPS for smooth control
            
        except Exception as e:
            print(f"❌ Error in manual control thread: {e}")
            time.sleep(0.1)

def detection_thread():
    """Detection thread - keeping from original but simplified"""
    global stop_sign_active, stop_sign_start_time
    
    print("🔍 Detection Thread Started!")
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
                
                # Stop sign logic
                if is_dur and not stop_sign_active:
                    print("🛑🛑🛑 STOP SIGN DETECTED! Starting 3-second stop...")
                    stop_sign_active = True
                    stop_sign_start_time = time.time()
                
                # Check if stop duration is over
                if stop_sign_active:
                    elapsed = time.time() - stop_sign_start_time
                    if elapsed >= STOP_DURATION:
                        print("✅ Stop duration complete! Resuming...")
                        stop_sign_active = False
                    else:
                        remaining = STOP_DURATION - elapsed
                        print(f"⏱️ STOP ACTIVE: {remaining:.1f}s remaining")
                
                # Add visual information to frame
                mode_text = "MANUAL" if manual_control_active else "AUTO"
                mode_color = (0, 255, 0) if manual_control_active else (0, 0, 255)
                
                cv2.putText(frame, f"Mode: {mode_text}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, mode_color, 2)
                cv2.putText(frame, f"Detected: {levha}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.putText(frame, f"Throttle: {current_throttle:.2f}", (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
                cv2.putText(frame, f"Steer: {current_steer:.2f}", (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
                cv2.putText(frame, f"Brake: {current_brake:.2f}", (20, 250), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
                
                # Control instructions
                cv2.putText(frame, "Controls: WASD + SPACE(brake) + M(mode) + Q(quit)", 
                           (20, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                
                # Stop sign visual feedback
                if stop_sign_active:
                    elapsed_time = time.time() - stop_sign_start_time
                    remaining_time = STOP_DURATION - elapsed_time
                    if remaining_time > 0:
                        cv2.rectangle(frame, (10, 350), (800, 410), (0, 0, 255), -1)
                        cv2.rectangle(frame, (10, 350), (800, 410), (255, 255, 255), 3)
                        cv2.putText(frame, f"🛑 STOP SIGN ACTIVE - WAITING: {remaining_time:.1f}s", 
                                   (20, 390), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
                
                # Display the frame
                cv2.imshow('CARLA Manual Control - WASD', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_threads()
                    break
            
            time.sleep(0.1)  # Small delay
            
        except Exception as e:
            print(f"❌ Error in detection thread: {e}")
            time.sleep(0.1)

def control_thread():
    """Control thread - applies manual or auto control"""
    global manual_control_active, current_throttle, current_steer, current_brake
    
    print("🚗 Control Thread Started!")
    
    while not stop_event.is_set():
        try:
            # Update global position
            transform = vehicle.get_transform()
            Global.current_x = transform.location.x
            Global.current_y = transform.location.y
            Global.yaw = transform.rotation.yaw
            
            # Update direction
            from Controller.yon import get_direction
            get_direction()
            
            if manual_control_active:
                # MANUAL CONTROL MODE
                throttle = current_throttle
                steer = current_steer
                brake = current_brake
                
                # Apply control to vehicle
                control_command = carla.VehicleControl(
                    throttle=max(0.0, throttle),  # Ensure positive throttle
                    steer=steer,
                    brake=brake,
                    reverse=(throttle < 0.0)  # Enable reverse if throttle is negative
                )
                vehicle.apply_control(control_command)
                
                # Set turn signals based on steering
                if steer > 0.3:
                    set_turn_signal(vehicle, left_signal=False, right_signal=True)
                elif steer < -0.3:
                    set_turn_signal(vehicle, left_signal=True, right_signal=False)
                else:
                    set_turn_signal(vehicle, left_signal=False, right_signal=False)
                
            else:
                # AUTO CONTROL MODE (simplified from original)
                try:
                    # Use the original controllers (simplified)
                    throttle = s_c.update(vehicle) if hasattr(s_c, 'update') else 0.3
                    steer = 0.0  # Simplified - just go straight in auto mode
                    brake = Global.breaking if hasattr(Global, 'breaking') else 0.0
                    
                    # Apply control
                    control_command = carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)
                    vehicle.apply_control(control_command)
                    
                    print(f"🤖 AUTO: throttle={throttle:.2f}, steer={steer:.2f}, brake={brake:.2f}")
                    
                except Exception as e:
                    print(f"❌ Error in auto control: {e}")
                    # Fallback to safe stop
                    vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
            
            time.sleep(0.05)  # 20 FPS control loop
            
        except Exception as e:
            print(f"❌ Error in control thread: {e}")
            time.sleep(0.1)

def main():
    """Main function"""
    print("🚀 Starting CARLA Manual Control System...")
    print("🎮 Use WASD keys to control the vehicle!")
    
    # Start threads
    manual_thread = threading.Thread(target=manual_control_thread)
    detection_th = threading.Thread(target=detection_thread)
    control_th = threading.Thread(target=control_thread)
    
    manual_thread.start()
    detection_th.start()
    control_th.start()
    
    try:
        # Keep main thread alive
        while not stop_event.is_set():
            time.sleep(1)
            if manual_control_active:
                print("🎮 MANUAL MODE - Use WASD keys")
            else:
                print("🤖 AUTO MODE - Press M to switch to manual")
                
    except KeyboardInterrupt:
        print("🛑 Stopping system...")
        stop_threads()
    
    # Wait for threads to finish
    manual_thread.join()
    detection_th.join()
    control_th.join()
    
    # Cleanup
    try:
        pygame.quit()
        if camera.is_listening:
            camera.stop()
        if camera.is_alive:
            camera.destroy()
        if vehicle.is_alive:
            vehicle.destroy()
        cv2.destroyAllWindows()
        print("✅ Cleanup complete")
    except Exception as e:
        print(f"❌ Error during cleanup: {e}")

if __name__ == "__main__":
    main()
