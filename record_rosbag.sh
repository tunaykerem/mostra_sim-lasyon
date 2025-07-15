#!/bin/bash

# CARLA ROS2 Rosbag Recording Script
# Bu script CARLA simülasyonundan rosbag kaydetmek için kullanılır

# Renkli çıktı için
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== CARLA ROS2 Rosbag Recording Script ===${NC}"

# ROS2 ortamını kontrol et
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}ROS2 ortamı bulunamadı. ROS2'yi source edin:${NC}"
    echo "source /opt/ros/humble/setup.bash"
    exit 1
fi

echo -e "${GREEN}ROS2 Distro: $ROS_DISTRO${NC}"

# Kayıt klasörü oluştur
RECORD_DIR="rosbag_recordings"
mkdir -p $RECORD_DIR

# Zaman damgası ile dosya adı oluştur
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="carla_recording_$TIMESTAMP"

echo -e "${YELLOW}Kayıt dosyası: $RECORD_DIR/$BAG_NAME${NC}"

# Kullanıcıya seçenek sun
echo ""
echo "Hangi kayıt modunu tercih ediyorsunuz?"
echo "1) Tüm topic'leri kaydet (en fazla veri)"
echo "2) Sadece önemli topic'leri kaydet (optimize)"
echo "3) Sadece sensör verilerini kaydet (minimal)"
echo "4) Özel topic listesi"

read -p "Seçiminizi yapın (1-4): " choice

case $choice in
    1)
        echo -e "${GREEN}Tüm topic'ler kaydediliyor...${NC}"
        ros2 bag record -a -o $RECORD_DIR/$BAG_NAME
        ;;
    2)
        echo -e "${GREEN}Önemli topic'ler kaydediliyor...${NC}"
        ros2 bag record \
            /odom \
            /imu_raw \
            /traffic_sign \
            /cone_detection \
            /avoidance_status \
            /vehicle_control \
            /speed \
            /trajectory_status \
            /vehicle_status \
            /turn_signals \
            -o $RECORD_DIR/$BAG_NAME
        ;;
    3)
        echo -e "${GREEN}Sadece sensör verileri kaydediliyor...${NC}"
        ros2 bag record \
            /odom \
            /imu_raw \
            /speed \
            -o $RECORD_DIR/$BAG_NAME
        ;;
    4)
        echo -e "${YELLOW}Mevcut topic'ler:${NC}"
        ros2 topic list
        echo ""
        read -p "Kaydetmek istediğiniz topic'leri boşlukla ayırarak girin: " custom_topics
        echo -e "${GREEN}Özel topic'ler kaydediliyor: $custom_topics${NC}"
        ros2 bag record $custom_topics -o $RECORD_DIR/$BAG_NAME
        ;;
    *)
        echo -e "${RED}Geçersiz seçim!${NC}"
        exit 1
        ;;
esac

echo -e "${GREEN}Kayıt tamamlandı: $RECORD_DIR/$BAG_NAME${NC}"

# Kayıt hakkında bilgi ver
echo ""
echo -e "${YELLOW}Kayıt bilgileri:${NC}"
ros2 bag info $RECORD_DIR/$BAG_NAME

echo ""
echo -e "${GREEN}Kayıtı oynatmak için:${NC}"
echo "ros2 bag play $RECORD_DIR/$BAG_NAME"
