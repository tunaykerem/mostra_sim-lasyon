#!/usr/bin/env python3
"""
Gelişmiş WASD Manuel Kontrol Sistemi - Pygame ile
"""

import cv2
from setup import setup
from webcam1 import TrafficSignDetector
from Util import Global
from Controller import speed_controller
from Controller import lateral_controller as lc
from Util import Referans, State
import sys
import glob
import detection.detect as dt1
import detection.cone_detector as cd
import detection.torch_detector as torch_det
import math
try:
    sys.path.append(glob.glob('../../carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg')[0])
except IndexError:
    pass
import carla
import threading
import time
import numpy as np
import pygame  # Pygame kullanımı

from FinalPathGenerator import Levha
from FinalPathGenerator import setLabel
from FinalPathGenerator import PathGenerator as pg
from FinalPathGenerator import Main
from Util import Global
from Controller.waypoint_interpolation import WaypointInterpolation

# Pygame başlatma (klavye kontrolü için)
pygame.init()
pygame.display.set_caption("CARLA Kontrol")
pygame.display.set_mode((100, 100), pygame.HIDDEN)  # Gizli pencere, sadece klavye girişi için

# Araç ve kamera
vehicle, camera, camera_data = setup()
s_c = speed_controller.DrivingController()

# Kontrol parametreleri
INTERP_LOOKAHEAD_DISTANCE = 1
INTERP_DISTANCE_RES = 0.01
closest_index = 0
closest_distance = 0
stop_event = threading.Event()
control_start_event = threading.Event()
trajectory_update_event = threading.Event()
trajectory_lock = threading.Lock()
keyboard_lock = threading.Lock()

# Dur işareti değişkenleri
stop_sign_active = False
stop_sign_start_time = 0
STOP_DURATION = 3.0

# Manuel kontrol değişkenleri
manual_control = True  # Başlangıçta manuel mod aktif
current_throttle = 0.0
current_steer = 0.0
current_brake = 0.0

# Yörünge başlatma
Global.x_main, Global.y_main = Main.getFinalTrajectory()
Global.stop_points = [[Global.x_main[-1], Global.y_main[-1]]]
Global.control_thread = True

def set_turn_signal(vehicle, left_signal=False, right_signal=False):
    """Dönüş sinyallerini ayarla"""
    try:
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
    except Exception as e:
        print(f"Sinyal hatası: {e}")

def stop_threads():
    """Tüm thread'leri durdur"""
    stop_event.set()
    pygame.quit()

def keyboard_control_thread():
    """Klavye kontrolü için thread"""
    global manual_control, current_throttle, current_steer, current_brake
    
    print("🎮 Klavye kontrol thread'i başladı!")
    print("🎮 Kontroller:")
    print("   W - İleri (gaz)")
    print("   S - Geri (fren/geri)")
    print("   A - Sola dön")
    print("   D - Sağa dön")
    print("   SPACE - Acil fren")
    print("   M - Manuel/Otomatik mod değiştir")
    print("   Q - Çıkış")
    
    clock = pygame.time.Clock()
    
    while not stop_event.is_set():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stop_threads()
                break
                
            # Manuel/Otomatik mod değiştirme
            if event.type == pygame.KEYDOWN and event.key == pygame.K_m:
                with keyboard_lock:
                    manual_control = not manual_control
                    current_throttle = 0.0
                    current_steer = 0.0
                    current_brake = 0.0
                print(f"{'🎮 MANUEL' if manual_control else '🤖 OTOMATİK'} mod aktif!")
                
            # Çıkış tuşu
            if event.type == pygame.KEYDOWN and event.key == pygame.K_q:
                print("🛑 Çıkış tuşuna basıldı, program kapatılıyor...")
                stop_threads()
                break
        
        # Manuel modda WASD kontrolleri
        if manual_control:
            keys = pygame.key.get_pressed()
            
            with keyboard_lock:
                # Gaz/Fren kontrolü (W/S)
                if keys[pygame.K_w]:
                    current_throttle = min(1.0, current_throttle + 0.05)
                    current_brake = 0.0
                    print(f"🚗 İleri: {current_throttle:.2f}")
                elif keys[pygame.K_s]:
                    # Hıza göre fren veya geri vites
                    velocity = vehicle.get_velocity()
                    speed = (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
                    if speed > 0.5:  # İleri gidiyorsa, önce fren
                        current_brake = min(1.0, current_brake + 0.1)
                        current_throttle = 0.0
                        print(f"🛑 Fren: {current_brake:.2f}")
                    else:  # Durmuşsa, geri vites
                        current_throttle = -0.5
                        current_brake = 0.0
                        print("🔄 Geri")
                else:
                    # Tuş basılı değilse yavaşça azalt
                    current_throttle = max(0.0, current_throttle - 0.03)
                    current_brake = max(0.0, current_brake - 0.1)
                
                # Direksiyon kontrolü (A/D)
                if keys[pygame.K_a]:
                    current_steer = max(-0.9, current_steer - 0.05)
                    print(f"⬅️ Sola: {current_steer:.2f}")
                elif keys[pygame.K_d]:
                    current_steer = min(0.9, current_steer + 0.05)
                    print(f"➡️ Sağa: {current_steer:.2f}")
                else:
                    # Merkeze doğru yavaşça düzelt
                    if current_steer > 0:
                        current_steer = max(0.0, current_steer - 0.05)
                    else:
                        current_steer = min(0.0, current_steer + 0.05)
                
                # Acil fren - Space tuşu
                if keys[pygame.K_SPACE]:
                    current_brake = 1.0
                    current_throttle = 0.0
                    print("🛑 ACİL FREN!")
            
            # Kontrol değerlerini ekrana yaz
            print(f"🎮 KONTROL: Gaz={current_throttle:.2f}, Dönüş={current_steer:.2f}, Fren={current_brake:.2f}")
        
        clock.tick(20)  # 20 FPS - CPU kullanımını azaltmak için

def detection_thread():
    """Nesne tespiti ve yol trafik işaretlerini algılayan thread"""
    global stop_sign_active, stop_sign_start_time
    
    print("🔍 Tespit thread'i başladı!")
    detector = torch_det.SimpleYOLOv8Detector('last.pt')
    cone_detector = cd.ConeDetector()
    
    # Koni engelleri için
    import detection.cone_avoidance as ca
    cone_avoidance = ca.ConeAvoidanceSystem()
    
    while not stop_event.is_set():
        try:
            # Kamera görüntüsünü al
            frame = camera_data['image'][:, :, :-1]
            frame = frame.astype(np.uint8)
            
            # YOLOv8 ile işareteri tespit et
            start_time = time.time()
            result = detector.process_frame(frame)
            
            if isinstance(result, tuple) and len(result) >= 2:
                frame, levha = result[0], result[1]
                
                print(f"🔍 YOLOv8 Tespiti: '{levha}'")
                
                # Dur işareti algılama (durak değil)
                is_dur = False
                if levha and levha != "None":
                    levha_clean = str(levha).strip().lower()
                    
                    if levha_clean == "dur":
                        is_dur = True
                        print(f"✅ TAM DUR EŞLEŞMESİ: '{levha}'")
                    elif "durak" in levha_clean:
                        print(f"🚌 DURAK tespit edildi (yok sayılıyor): '{levha}'")
                    elif "dur" in levha_clean and "durak" not in levha_clean:
                        is_dur = True
                        print(f"⚠️ KISMİ DUR EŞLEŞMESİ: '{levha}'")
                
                # Manuel modda değilse dur işareti uygulaması
                with keyboard_lock:
                    is_manual = manual_control
                
                if is_dur and not is_manual:
                    if not stop_sign_active:
                        print("🛑🛑🛑 DUR İŞARETİ TESPİT EDİLDİ! 3 saniye bekleniyor...")
                        stop_sign_active = True
                        stop_sign_start_time = time.time()
                        Global.breaking = 1.0
                
                # Dur işareti süresini kontrol et
                if stop_sign_active and not is_manual:
                    elapsed = time.time() - stop_sign_start_time
                    if elapsed >= STOP_DURATION:
                        print("✅ Bekleme süresi tamamlandı! Devam ediliyor...")
                        stop_sign_active = False
                        Global.breaking = 0.0
                    else:
                        remaining = STOP_DURATION - elapsed
                        print(f"⏱️ DUR AKTİF: {remaining:.1f}s kaldı")
                        Global.breaking = 1.0
            else:
                levha = "None"
                if result is not None:
                    frame = result
            
            # Koni tespiti (YOLO tarafından zaten koni tespit edilmemişse)
            yolo_detected_cone = (levha.lower() in ["koni", "cone"]) if levha != "None" else False
            
            cone_positions = []
            if not yolo_detected_cone:
                # Geleneksel koni dedektörünü kullan
                cone_positions, frame = cone_detector.detect_cones(frame)
                
                # Koni pozisyonlarını engelden kaçınma sistemi ile işle
                obstacle_changed, obstacle_data = cone_avoidance.process_cone_detections(cone_positions)
            else:
                # YOLO koni tespit ettiyse, engelden kaçınma için işle
                cone_positions = [[0, 3.0]]  # Koni 3m ileride, merkezde
                obstacle_changed, obstacle_data = cone_avoidance.process_cone_detections(cone_positions)
                print(f"🔴 YOLO koni tespit etti: {levha}")
            
            # Manuel modda değilse, engel durumu değiştiyse yörüngeyi güncelle
            with keyboard_lock:
                is_manual = manual_control
            
            if obstacle_changed and not is_manual:
                print(f"🚨 Engel durumu değişti ({len(cone_positions)} koni), yörünge yeniden oluşturuluyor...")
                
                if Global.dubaVar:
                    try:
                        # Engelden kaçınma yörüngesi oluştur
                        current_x = Global.current_x
                        current_y = Global.current_y
                        direction = Global.direction
                        
                        x_avoid, y_avoid = cone_avoidance.generate_avoidance_path(
                            current_x, current_y, direction)
                        
                        if x_avoid and y_avoid and len(x_avoid) > 0:
                            with trajectory_lock:
                                Global.x_main, Global.y_main = x_avoid, y_avoid
                                Global.avoidance_active = True
                            
                            trajectory_update_event.set()
                            print(f"✅ Yeni kaçış yörüngesi oluşturuldu ({len(Global.x_main)} nokta)")
                        else:
                            print("❌ Geçerli kaçış yörüngesi bulunamadı - ACİL DURUM YEDEK PLANI")
                            # Basit acil durum kaçış planı
                            lateral_offset = 3.0
                            x_points = []
                            y_points = []
                            
                            if direction == "K":  # Kuzey
                                x_points = [current_x, current_x - lateral_offset, current_x - lateral_offset, current_x]
                                y_points = [current_y, current_y + 2, current_y + 10, current_y + 15]
                            elif direction == "G":  # Güney
                                x_points = [current_x, current_x + lateral_offset, current_x + lateral_offset, current_x]
                                y_points = [current_y, current_y - 2, current_y - 10, current_y - 15]
                            elif direction == "D":  # Doğu
                                x_points = [current_x, current_x + 2, current_x + 10, current_x + 15]
                                y_points = [current_y, current_y + lateral_offset, current_y + lateral_offset, current_y]
                            elif direction == "B":  # Batı
                                x_points = [current_x, current_x - 2, current_x - 10, current_x - 15]
                                y_points = [current_y, current_y - lateral_offset, current_y - lateral_offset, current_y]
                            
                            with trajectory_lock:
                                Global.x_main, Global.y_main = x_points, y_points
                                Global.avoidance_active = True
                            
                            trajectory_update_event.set()
                            print(f"🆘 ACİL DURUM kaçış yolu oluşturuldu ({len(x_points)} nokta)")
                    except Exception as e:
                        print(f"❌ Yörünge yeniden oluşturma hatası: {e}")
                        
                        # Son çare olarak orijinal yörüngeyi kullan
                        with trajectory_lock:
                            if not Global.x_main or len(Global.x_main) == 0:
                                try:
                                    Global.x_main, Global.y_main = Main.getFinalTrajectory()
                                    print(f"⚠️ Orijinal yörüngeye dönülüyor ({len(Global.x_main)} nokta)")
                                except Exception as e2:
                                    print(f"💥 KRİTİK: Final yörünge alınamadı: {e2}")
                        
                        trajectory_update_event.set()
                else:
                    # Engel yok, ama daha önce kaçınma modundaydık, normal yörüngeye dön
                    if getattr(Global, 'avoidance_active', False):
                        try:
                            with trajectory_lock:
                                Global.x_main, Global.y_main = Main.getFinalTrajectory()
                                Global.avoidance_active = False
                            
                            trajectory_update_event.set()
                            print(f"✅ Normal yörüngeye dönüldü ({len(Global.x_main)} nokta)")
                        except Exception as e:
                            print(f"❌ Normal yörüngeye dönüş hatası: {e}")
            
            # FPS ve zaman bilgisi
            elapsed_time = time.time() - start_time
            fps = 1.0 / elapsed_time if elapsed_time > 0 else 0
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            
            # Ekrana bilgileri ekle
            cv2.putText(frame, f"FPS: {fps:.1f}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.putText(frame, f"Tespit: {levha}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.putText(frame, f"Poz: ({Global.current_x:.1f}, {Global.current_y:.1f}) Yön: {Global.direction}", 
                       (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
            
            # Kontrol modu göstergesi
            with keyboard_lock:
                mode_text = "MANUEL KONTROL (WASD)" if manual_control else "OTOMATİK KONTROL"
                mode_color = (0, 0, 255) if manual_control else (0, 255, 0)
            
            cv2.rectangle(frame, (10, 180), (600, 230), mode_color, -1)
            cv2.putText(frame, mode_text, (20, 215), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
            
            # Manuel modda kontrol değerlerini göster
            with keyboard_lock:
                is_manual = manual_control
                t_val = current_throttle
                s_val = current_steer
                b_val = current_brake
            
            if is_manual:
                control_text = f"Gaz: {t_val:.2f} | Direksiyon: {s_val:.2f} | Fren: {b_val:.2f}"
                cv2.putText(frame, control_text, (20, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(frame, "M tuşu ile otomatik moda geç", (20, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            else:
                # Otomatik modda dur işareti göstergesi
                if stop_sign_active:
                    elapsed_time = time.time() - stop_sign_start_time
                    remaining_time = STOP_DURATION - elapsed_time
                    if remaining_time > 0:
                        cv2.rectangle(frame, (10, 240), (800, 300), (0, 0, 255), -1)
                        cv2.rectangle(frame, (10, 240), (800, 300), (255, 255, 255), 3)
                        cv2.putText(frame, f"🛑 DUR LEVHASİ AKTİF - BEKLEME: {remaining_time:.1f}s", 
                                  (20, 280), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
                        
                        cv2.putText(frame, f"Tespit edilen: {levha}", 
                                  (20, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                else:
                    cv2.putText(frame, "M tuşu ile manuel moda geç", (20, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # Koni tespit bilgisi
            base_y_pos = 320 if is_manual else 360
            if len(cone_positions) > 0:
                closest_cone = min(cone_positions, key=lambda pos: math.sqrt(pos[0]**2 + pos[1]**2))
                cv2.putText(frame, f"KONİ TESPİT EDİLDİ: {closest_cone[1]:.1f}m ileride, {closest_cone[0]:.1f}m offset", 
                           (20, base_y_pos), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                
                # Engelden kaçınma uyarısı
                if Global.dubaVar and not is_manual:
                    warning_y_start = base_y_pos + 20
                    warning_y_end = warning_y_start + 50
                    cv2.rectangle(frame, (10, warning_y_start), (600, warning_y_end), (0, 0, 255), -1)
                    cv2.putText(frame, "⚠️ ENGELDEN KAÇINMA AKTİF ⚠️", 
                               (20, warning_y_start + 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
                    
                    # Engel detayları
                    obstacle_details = f"Engel verisi: {Global.dubaEngel}"
                    cv2.putText(frame, obstacle_details, (20, warning_y_end + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    
                    # Mevcut kaçınma yolu bilgisi
                    if hasattr(Global, 'avoidance_path') and Global.avoidance_path:
                        path_info = f"Kaçınma yolu: {len(Global.avoidance_path)} nokta"
                        cv2.putText(frame, path_info, (20, warning_y_end + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # Timestamp
            timestamp_y = 500 if len(cone_positions) > 0 else 450
            cv2.putText(frame, timestamp, (20, timestamp_y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            
            # Çıkış bilgisi
            cv2.putText(frame, "Çıkış için Q tuşuna basın", (20, timestamp_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Görüntüyü göster
            cv2.imshow('CARLA Kamera - Q:Çıkış, M:Mod Değiştir, WASD:Kontrol', frame)
            
            # Otomatik modda işaret bilgisiyle global durumu güncelle
            if not is_manual and levha != "None":
                # Önceki yörüngeyi sakla (karşılaştırma için)
                with trajectory_lock:
                    prev_x = Global.x_main.copy() if hasattr(Global, 'x_main') and Global.x_main else []
                    prev_y = Global.y_main.copy() if hasattr(Global, 'y_main') and Global.y_main else []
                
                # Levha bilgisiyle güncelle
                Levha.levha(levha, Global.direction, Global.current_x, Global.current_y)
                
                # Yörünge değiştiyse thread 2'yi uyar
                with trajectory_lock:
                    if (obstacle_changed or 
                        not prev_x or not prev_y or 
                        len(Global.x_main) != len(prev_x) or 
                        len(Global.y_main) != len(prev_y) or
                        not np.array_equal(Global.x_main, prev_x) or 
                        not np.array_equal(Global.y_main, prev_y)):
                        
                        print("Yörünge güncellendi! Kontrol thread'i bilgilendiriliyor...")
                        trajectory_update_event.set()
            
            # Thread2'nin başlaması için işaret ver
            if Global.control_thread:
                control_start_event.set()
                Global.control_thread = False
            
            # 'q' ile çıkış
            key = cv2.waitKey(1)
            if key == ord('q'):
                print("Q tuşuna basıldı, araç ve kamera siliniyor...")
                stop_threads()
                break
        
        except Exception as e:
            print(f"❌ Tespit thread'inde hata: {e}")
            time.sleep(0.1)

def control_thread():
    """Araç kontrol thread'i"""
    print("🚗 Kontrol thread'i başladı - sinyal bekleniyor")
    
    # Dur işareti değişkenleri
    global stop_sign_active, stop_sign_start_time
    
    control_start_event.wait()  # Sinyal gelene kadar bekle
    print("🚗 Kontrol döngüsü başladı")
    
    # Waypoint'leri başlat
    with trajectory_lock:
        current_waypoints = np.array(list(zip(Global.x_main, Global.y_main)))
    
    print(f"İlk kontrol noktaları: {len(current_waypoints)} nokta")
    if len(current_waypoints) > 0:
        print(f"İlk nokta: {current_waypoints[0]}")
        if len(current_waypoints) > 1:
            print(f"Son nokta: {current_waypoints[-1]}")
    
    while not stop_event.is_set():
        # Manuel kontrol modunu kontrol et
        with keyboard_lock:
            is_manual = manual_control
            man_throttle = current_throttle
            man_steer = current_steer
            man_brake = current_brake
        
        # Manuel modda doğrudan kullanıcı kontrollerini uygula
        if is_manual:
            control_command = carla.VehicleControl(
                throttle=man_throttle if man_throttle >= 0 else 0,
                steer=man_steer,
                brake=man_brake if man_throttle >= 0 else 0,
                reverse=True if man_throttle < 0 else False
            )
            vehicle.apply_control(control_command)
            
            # Dönüş sinyallerini direksiyon açısına göre ayarla
            if man_steer > 0.3:
                set_turn_signal(vehicle, left_signal=False, right_signal=True)
            elif man_steer < -0.3:
                set_turn_signal(vehicle, left_signal=True, right_signal=False)
            else:
                set_turn_signal(vehicle, left_signal=False, right_signal=False)
            
            # Manuel modda global konum bilgilerini güncelle
            location = vehicle.get_location()
            Global.current_x = location.x
            Global.current_y = location.y
            transform = vehicle.get_transform()
            Global.yaw = transform.rotation.yaw
            
            # Araç yönünü güncelle
            from Controller.yon import get_direction
            get_direction()
            
            time.sleep(0.02)
            continue
        
        # OTOMATİK KONTROL MODU aşağıda
        # Yörünge güncellemelerini kontrol et
        if trajectory_update_event.is_set():
            print("🔄 Yörünge güncellemesi tespit edildi - kontrol noktaları güncelleniyor...")
            with trajectory_lock:
                # x_main ve y_main geçerli mi kontrol et
                if (hasattr(Global, 'x_main') and hasattr(Global, 'y_main') and 
                    Global.x_main is not None and Global.y_main is not None and
                    len(Global.x_main) > 0 and len(Global.y_main) > 0):
                    
                    # Global yörüngeden waypoint'leri HEMEN güncelle
                    current_waypoints = np.array(list(zip(Global.x_main, Global.y_main)))
                    print(f"✅ Waypoint'ler güncellendi: {len(current_waypoints)} nokta")
                    
                    # Yeni yörüngenin ilk ve son noktalarını yazdır
                    if len(current_waypoints) > 0:
                        print(f"İlk nokta: {current_waypoints[0]}")
                        if len(current_waypoints) > 1:
                            print(f"Son nokta: {current_waypoints[-1]}")
                    
                    # Kaçınma modunda mıyız kontrol et
                    if getattr(Global, 'avoidance_active', False):
                        print("⚠️ KAÇINMA MODU AKTİF - kaçınma yörüngesi takip ediliyor")
                else:
                    print("❌ Uyarı: Global'de geçersiz x_main veya y_main, önceki waypoint'ler korunuyor")
            
            # İşlendikten sonra event'i temizle
            trajectory_update_event.clear()
        
        # Geçerli waypoint'lerimiz olduğundan emin ol
        if len(current_waypoints) == 0:
            print("Geçerli waypoint yok, yeniden oluşturmaya çalışılıyor")
            try:
                with trajectory_lock:
                    Global.x_main, Global.y_main = Main.getFinalTrajectory()
                    current_waypoints = np.array(list(zip(Global.x_main, Global.y_main)))
                print(f"Waypoint'ler yeniden oluşturuldu: {len(current_waypoints)} nokta")
            except Exception as e:
                print(f"❌ Waypoint yeniden oluşturma hatası: {e}")
                time.sleep(0.1)
                continue
        
        # Mevcut waypoint'leri kontrol için kullan
        try:
            wp_interp, wp_distance, wp_interp, wp_interp_hash = WaypointInterpolation.interpolate_waypoints(current_waypoints, INTERP_DISTANCE_RES)
        except Exception as e:
            print(f"❌ Waypoint interpolasyon hatası: {e}")
            time.sleep(0.1)
            continue
        
        location = vehicle.get_location()
        
        # Global konum değişkenlerini güncelle
        Global.current_x = location.x
        Global.current_y = location.y
        
        velocity = vehicle.get_velocity()
        transform = vehicle.get_transform()
        Global.yaw = transform.rotation.yaw
        
        # Aracın yönünü yaw'a göre güncelle
        from Controller.yon import get_direction
        get_direction()
        
        try:
            # Araç kontrollerini al
            throttle = s_c.update(vehicle)
            steer = lc.update(vehicle, current_waypoints, wp_distance, wp_interp, wp_interp_hash, INTERP_LOOKAHEAD_DISTANCE, closest_index, closest_distance)
            
            # Dur işareti kontrol mantığı
            if stop_sign_active:
                # ZORLA DURDUR - her şeyi geçersiz kıl, Global.breaking kullan
                throttle = 0.0
                Global.breaking = 1.0
                print(f"🛑 ZORLA DURDURMA: fren={Global.breaking}, gaz={throttle}")
            else:
                # Normal sürüş - Global.breaking ayarına uy
                if Global.breaking > 0:
                    print(f"🚗 Normal w/ fren: fren={Global.breaking}, gaz={throttle}")
                else:
                    print(f"🚗 Normal sürüş: fren={Global.breaking}, gaz={throttle}")
            
            # Kaçınma modunda güvenlik için hızı azalt
            if getattr(Global, 'avoidance_active', False):
                print(f"🐢 Kaçınma için hız azaltıldı (gaz: {throttle:.2f} -> {throttle * 0.7:.2f})")
                throttle *= 0.7
            
            # Araç kontrollerini uygula
            control_command = carla.VehicleControl(throttle=throttle, steer=steer, brake=Global.breaking)
            vehicle.apply_control(control_command)
            
            # Dönüş sinyallerini direksiyon açısına göre ayarla
            if steer > 0.1:
                set_turn_signal(vehicle, left_signal=False, right_signal=True)
            elif steer < -0.1:
                set_turn_signal(vehicle, left_signal=True, right_signal=False)
            else:
                set_turn_signal(vehicle, left_signal=False, right_signal=False)
        except Exception as e:
            print(f"❌ Kontrol döngüsünde hata: {e}")
            # Güvenli varsayılanları uygula
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
            set_turn_signal(vehicle, left_signal=False, right_signal=False)

        State.set_state()
        Referans.set_referance()
        
        # Kaçınma modunda değilse durma noktalarını yazdır
        if not getattr(Global, 'avoidance_active', False):
            print(Global.stop_points)

def main():
    """Ana program"""
    print("=================================================")
    print("CARLA WASD Kontrol + Otomatik Sürüş Sistemi")
    print("=================================================")
    print("Kontroller:")
    print("  M     - Manuel/Otomatik mod değiştir")
    print("  W     - İleri (gaz)")
    print("  S     - Fren/Geri")
    print("  A     - Sola dön")
    print("  D     - Sağa dön")
    print("  SPACE - Acil fren")
    print("  Q     - Çıkış")
    print("=================================================")
    print("MANUEL modda başlıyor. Otomatik moda geçmek için M tuşuna basın.")
    print("=================================================")
    
    # Thread'leri oluştur
    kb_thread = threading.Thread(target=keyboard_control_thread)
    detect_thread = threading.Thread(target=detection_thread)
    ctrl_thread = threading.Thread(target=control_thread)
    
    # Thread'leri başlat
    kb_thread.start()
    detect_thread.start()
    ctrl_thread.start()
    
    try:
        # Ana thread aktif tut
        while not stop_event.is_set():
            time.sleep(1)
            with keyboard_lock:
                mode_text = "MANUEL" if manual_control else "OTOMATİK"
            print(f"🚗 {mode_text} MOD AKTIF - Çıkış için Q tuşuna basın")
    except KeyboardInterrupt:
        print("\n⚠️ Klavye kesintisi, kapatılıyor...")
        stop_threads()
    
    # Thread'lerin bitmesini bekle
    kb_thread.join()
    detect_thread.join()
    ctrl_thread.join()
    
    # Kaynakları temizle
    print("Kaynaklar temizleniyor...")
    if 'camera' in globals() and camera.is_listening:
        camera.stop()
    if 'camera' in globals() and camera.is_alive:
        camera.destroy()
    if 'vehicle' in globals() and vehicle.is_alive:
        vehicle.destroy()
    cv2.destroyAllWindows()
    pygame.quit()
    print("Temizlik tamamlandı.")

if __name__ == "__main__":
    main()
