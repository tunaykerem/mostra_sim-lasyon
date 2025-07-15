# CARLA ROS2 Data Recording Setup

Bu proje CARLA simülasyonundan ROS2 topic'leri olarak veri yayınlar ve rosbag ile kayıt alınmasını sağlar.

## Gereksinimler

1. ROS2 (Humble/Foxy/Galactic)
2. CARLA 0.9.15
3. Python paketleri:
   - rclpy
   - sensor_msgs
   - std_msgs
   - nav_msgs
   - geometry_msgs
   - tf_transformations

## ROS2 Kurulumu

```bash
# ROS2 Humble kurulumu (Ubuntu 22.04)
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 repository ekle
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 kurulum
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-humble-cv-bridge

# Ortam değişkenlerini ayarla
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Yayınlanan ROS2 Topic'ler

Aşağıdaki topic'ler main3.py tarafından yayınlanır:

### Sensör Verileri
- `/odom` (nav_msgs/Odometry) - Araç odometrisi (konum, hız, yön)
- `/imu_raw` (sensor_msgs/Imu) - IMU verileri (ivme, açısal hız, yönelim)
- `/camera_info` (sensor_msgs/CameraInfo) - Kamera kalibrasyonu

### Araç Kontrol Verileri
- `/throttle` (std_msgs/Float32) - Gaz pedalı değeri
- `/steering_angle` (std_msgs/Float32) - Direksiyon açısı
- `/brake` (std_msgs/Float32) - Fren pedalı değeri
- `/speed` (std_msgs/Float32) - Araç hızı (km/h)
- `/vehicle_control` (std_msgs/String) - Tüm kontrol verileri (birleşik)

### Sürüş Verileri
- `/traffic_sign` (std_msgs/String) - Tespit edilen trafik işaretleri
- `/cone_detection` (std_msgs/String) - Koni tespiti bilgileri
- `/avoidance_status` (std_msgs/String) - Engel kaçınma durumu
- `/turn_signals` (std_msgs/String) - Sinyal durumu (LEFT/RIGHT/NONE)
- `/trajectory_status` (std_msgs/String) - Yörünge takip durumu
- `/vehicle_position` (std_msgs/String) - Araç konumu ve yönü
- `/current_waypoint` (std_msgs/String) - Mevcut waypoint bilgileri
- `/vehicle_status` (std_msgs/String) - Genel araç durumu

## Rosbag Kaydetme

### Tüm topic'leri kaydetmek için:
```bash
ros2 bag record -a
```

### Belirli topic'leri kaydetmek için:
```bash
ros2 bag record /odom /imu_raw /traffic_sign /cone_detection /vehicle_control /throttle /steering_angle /brake /speed
```

### Önemli topic'ler için optimize edilmiş kayıt:
```bash
ros2 bag record \
  /odom \
  /imu_raw \
  /traffic_sign \
  /cone_detection \
  /avoidance_status \
  /vehicle_control \
  /speed \
  /trajectory_status \
  /vehicle_status
```

## Kullanım

1. CARLA simülatörünü başlatın:
```bash
cd /path/to/carla
./CarlaUE4.sh
```

2. ROS2 ortamını ayarlayın:
```bash
source /opt/ros/humble/setup.bash
```

3. Ana programı çalıştırın:
```bash
cd /home/tunay/carla/PythonAPI/examples/dur_ve_engelden_kacis_kurca
python3 main3.py
```

4. Başka bir terminalde rosbag kaydını başlatın:
```bash
source /opt/ros/humble/setup.bash
ros2 bag record -a -o carla_recording_$(date +%Y%m%d_%H%M%S)
```

## Rosbag Analizi

Kaydedilen bag dosyalarını analiz etmek için:

```bash
# Bag dosyası hakkında bilgi al
ros2 bag info your_bag_file

# Topic'leri listele
ros2 topic list

# Belirli bir topic'i izle
ros2 topic echo /traffic_sign

# Bag dosyasını oynat
ros2 bag play your_bag_file
```

## Veri Formatları

### Traffic Sign Detection
```
traffic_sign: "dur", "sola_donulmez", "saga_donulmez", vb.
```

### Cone Detection
```
cones_detected:2,closest_distance:5.20,closest_offset:1.50
```

### Avoidance Status
```
avoidance_active:true,obstacle_type:cone,trajectory_points:150
```

### Vehicle Control
```
throttle:0.650,steer:-0.120,brake:0.000,speed:25.30
```

### Vehicle Status
```
speed:25.30,throttle:0.650,brake:0.000,steer:-0.120,stop_sign_active:false,cooldown_active:false
```

## Troubleshooting

1. **ROS2 topic'leri görünmüyor**: `ros2 topic list` ile kontrol edin
2. **CV Bridge hatası**: `sudo apt install ros-humble-cv-bridge` çalıştırın
3. **Import hataları**: ROS2 ortamının doğru şekilde source edildiğinden emin olun

## Notlar

- Tüm koordinatlar CARLA koordinat sistemindedir
- Odometry verileri 'odom' frame'inde yayınlanır
- IMU verileri 'imu_link' frame'inde yayınlanır
