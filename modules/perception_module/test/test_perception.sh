#!/bin/bash
# =============================================================================
# 🔥 Perception Module - Test Scripti 🔥
# Docker içinde çalıştırılmalıdır!
# Kullanım: bash test/test_perception.sh
# =============================================================================

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m'

pass_count=0
fail_count=0
warn_count=0
test_number=0
ONLY_TEST7=${ONLY_TEST7:-1}

pass() {
    echo -e "  ${GREEN}[PASS]${NC} $1"
    ((pass_count++))
}

fail() {
    echo -e "  ${RED}[FAIL]${NC} $1"
    ((fail_count++))
}

warn() {
    echo -e "  ${YELLOW}[WARN]${NC} $1"
    ((warn_count++))
}

info() {
    echo -e "  ${BLUE}[INFO]${NC} $1"
}

header() {
    ((test_number++))
    echo ""
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${MAGENTA}  TEST $test_number: $1${NC}"
    echo -e "${MAGENTA}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

cleanup_all() {
    info "Tüm arka plan süreçleri temizleniyor..."
    kill $SCAN_PID $TF1_PID $TF2_PID $LAUNCH_PID 2>/dev/null
    wait $SCAN_PID $TF1_PID $TF2_PID $LAUNCH_PID 2>/dev/null
    sleep 1
}

# Ctrl+C ile temiz çıkış
trap cleanup_all EXIT

echo ""
echo -e "${BOLD}${CYAN}╔═══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${CYAN}║    🔥 PERCEPTION MODULE - HAYVANÎ TEST SÜİTİ 🔥             ║${NC}"
echo -e "${BOLD}${CYAN}║    Alaz Otonom Araç - LaserScan → OccupancyGrid              ║${NC}"
echo -e "${BOLD}${CYAN}╚═══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  ${BLUE}Tarih:${NC} $(date '+%Y-%m-%d %H:%M:%S')"
echo -e "  ${BLUE}Host:${NC}  $(hostname)"
echo -e "  ${BLUE}ROS2:${NC}  $(printenv ROS_DISTRO 2>/dev/null || echo 'bilinmiyor')"

# =============================================================================
header "BUILD & INSTALL"
# =============================================================================

info "perception_module build ediliyor..."
cd /workspace
BUILD_OUTPUT=$(colcon build --packages-select perception_module 2>&1)
BUILD_EXIT=$?

if [ $BUILD_EXIT -eq 0 ]; then
    pass "colcon build başarılı (exit code: 0)"
else
    fail "colcon build BAŞARISIZ (exit code: $BUILD_EXIT)"
    echo "$BUILD_OUTPUT" | tail -10
    echo ""
    echo -e "${RED}Build hatası var, devam edilemiyor!${NC}"
    exit 1
fi

# Source workspace
source /workspace/install/setup.bash 2>/dev/null

# Paket bulunuyor mu?
if ros2 pkg prefix perception_module > /dev/null 2>&1; then
    PKG_PREFIX=$(ros2 pkg prefix perception_module)
    pass "perception_module paketi bulundu: $PKG_PREFIX"
else
    fail "perception_module paketi bulunamadı!"
    exit 1
fi

# =============================================================================
header "LAUNCH DOSYALARI KONTROLÜ"
# =============================================================================

PKG_DIR="$PKG_PREFIX/share/perception_module"

# Ana launch
if [ -f "$PKG_DIR/launch/perception.launch.xml" ]; then
    pass "perception.launch.xml ✓"
else
    fail "perception.launch.xml KURULMAMI"
fi

# PCL pipeline launch
if [ -f "$PKG_DIR/launch/laserscan_to_pcl_and_occ.launch.xml" ]; then
    pass "laserscan_to_pcl_and_occ.launch.xml ✓"
else
    fail "laserscan_to_pcl_and_occ.launch.xml KURULMAMI"
fi

# README
if [ -f "$PKG_DIR/README.md" ]; then
    pass "README.md ✓"
else
    warn "README.md kurulmamış"
fi

# Gereksiz dosya kalmamış mı?
if [ -f "$PKG_DIR/launch/laserscan_direct_occ.launch.py" ]; then
    warn "laserscan_direct_occ.launch.py hala kurulu (gereksiz dosya!)"
else
    pass "Gereksiz .py dosyası yok ✓"
fi

# =============================================================================
header "AUTOWARE BAĞIMLILIKLARI"
# =============================================================================

# autoware_probabilistic_occupancy_grid_map
if ros2 pkg prefix autoware_probabilistic_occupancy_grid_map > /dev/null 2>&1; then
    pass "autoware_probabilistic_occupancy_grid_map paketi mevcut"
else
    fail "autoware_probabilistic_occupancy_grid_map paketi YOK"
    exit 1
fi

# pointcloud_to_laserscan
if ros2 pkg prefix pointcloud_to_laserscan > /dev/null 2>&1; then
    pass "pointcloud_to_laserscan paketi mevcut"
else
    warn "pointcloud_to_laserscan paketi YOK (pointcloud pipeline çalışmaz)"
fi

# Executable mevcut mu?
EXEC_LIST=$(ros2 pkg executables autoware_probabilistic_occupancy_grid_map 2>/dev/null)
if echo "$EXEC_LIST" | grep -q "laserscan_based_occupancy_grid_map_node"; then
    pass "laserscan_based_occupancy_grid_map_node executable mevcut"
else
    fail "laserscan_based_occupancy_grid_map_node executable YOK"
fi

# Config dosyaları
AW_DIR=$(ros2 pkg prefix autoware_probabilistic_occupancy_grid_map)/share/autoware_probabilistic_occupancy_grid_map

declare -a CONFIG_FILES=(
    "config/laserscan_based_occupancy_grid_map.param.yaml"
    "config/binary_bayes_filter_updater.param.yaml"
)

for cfg in "${CONFIG_FILES[@]}"; do
    if [ -f "$AW_DIR/$cfg" ]; then
        pass "$(basename $cfg) ✓"
    else
        fail "$(basename $cfg) YOK"
    fi
done

# Composable node plugins
COMPONENTS=$(ros2 component types 2>/dev/null)
if echo "$COMPONENTS" | grep -q "LaserscanBasedOccupancyGridMapNode"; then
    pass "LaserscanBasedOccupancyGridMapNode plugin kayıtlı"
else
    warn "LaserscanBasedOccupancyGridMapNode plugin bulunamadı"
fi

if echo "$COMPONENTS" | grep -q "PointcloudBasedOccupancyGridMapNode"; then
    pass "PointcloudBasedOccupancyGridMapNode plugin kayıtlı"
else
    warn "PointcloudBasedOccupancyGridMapNode plugin YOK (pointcloud pipeline composable çalışmaz)"
fi

# =============================================================================
header "TF VE SCAN YAYINI"
# =============================================================================

info "TF publisher'lar başlatılıyor..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser &
TF1_PID=$!
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link &
TF2_PID=$!
sleep 2

# TF kontrolü
TF_FRAMES=$(ros2 topic echo /tf_static --once 2>/dev/null | head -20)
if [ -n "$TF_FRAMES" ]; then
    pass "TF static yayınları aktif"
else
    warn "TF static verisi alınamadı"
fi

info "Sahte LaserScan yayınlanıyor (20 ışın, 10 Hz)..."
ros2 topic pub /scan sensor_msgs/msg/LaserScan "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'laser'
  },
  angle_min: -1.57,
  angle_max: 1.57,
  angle_increment: 0.01,
  time_increment: 0.0,
  scan_time: 0.1,
  range_min: 0.1,
  range_max: 10.0,
  ranges: [5.0, 5.0, 5.0, 4.0, 3.0, 2.0, 1.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0],
  intensities: [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0]
}" --rate 10 &
SCAN_PID=$!
sleep 2

# Scan kontrolleri
SCAN_INFO=$(ros2 topic info /scan 2>/dev/null)
if echo "$SCAN_INFO" | grep -q "Publisher count: 1"; then
    pass "/scan publisher aktif"
else
    fail "/scan publisher YOK"
fi

SCAN_TYPE=$(ros2 topic info /scan 2>/dev/null | grep "Type:")
if echo "$SCAN_TYPE" | grep -q "sensor_msgs/msg/LaserScan"; then
    pass "/scan tipi doğru: sensor_msgs/msg/LaserScan"
else
    fail "/scan tipi YANLIŞ: $SCAN_TYPE"
fi

if timeout 3 ros2 topic echo /scan --once > /dev/null 2>&1; then
    pass "/scan mesajları alınıyor"
else
    fail "/scan mesajları ALINMIYOR"
fi

# Scan Hz kontrolü
info "/scan frekansı ölçülüyor (3 sn)..."
SCAN_HZ=$(timeout 3 ros2 topic hz /scan 2>&1 | grep "average rate" | head -1)
if [ -n "$SCAN_HZ" ]; then
    pass "/scan frekansı: $SCAN_HZ"
else
    warn "/scan frekansı ölçülemedi (kısa süre olabilir)"
fi

# =============================================================================
header "DIRECT PIPELINE — NODE BAŞLATMA"
# =============================================================================

info "Direct pipeline başlatılıyor: ros2 launch perception_module perception.launch.xml"

ros2 launch perception_module perception.launch.xml occ_pipeline:=direct 2>&1 &
LAUNCH_PID=$!

info "8 saniye bekleniyor (node başlatma)..."
sleep 8

# Node çalışıyor mu?
NODE_LIST=$(ros2 node list 2>/dev/null)
if echo "$NODE_LIST" | grep -q "occupancy_grid_map_node"; then
    pass "occupancy_grid_map_node ÇALIŞIYOR 🟢"
else
    fail "occupancy_grid_map_node başlatılamadı"
    info "Çalışan node'lar:"
    echo "$NODE_LIST" | while read line; do echo "    $line"; done
fi

# =============================================================================
header "DIRECT PIPELINE — TOPIC KONTROLLERI"
# =============================================================================

TOPIC_LIST=$(ros2 topic list 2>/dev/null)

# Beklenen topic'ler
declare -a EXPECTED_TOPICS=(
    "/scan"
    "/perception/occupancy_grid_map/map"
    "/tf_static"
)

for topic in "${EXPECTED_TOPICS[@]}"; do
    if echo "$TOPIC_LIST" | grep -q "$topic"; then
        pass "Topic mevcut: $topic"
    else
        fail "Topic YOK: $topic"
    fi
done

# OccGrid topic tipi
OCC_TYPE=$(ros2 topic info /perception/occupancy_grid_map/map 2>/dev/null | grep "Type:")
if echo "$OCC_TYPE" | grep -q "nav_msgs/msg/OccupancyGrid"; then
    pass "OccGrid tipi doğru: nav_msgs/msg/OccupancyGrid"
else
    fail "OccGrid tipi YANLIŞ: $OCC_TYPE"
fi

# OccGrid publisher var mı?
OCC_INFO=$(ros2 topic info /perception/occupancy_grid_map/map 2>/dev/null)
OCC_PUB_COUNT=$(echo "$OCC_INFO" | grep -oP 'Publisher count: \K[0-9]+' || echo "0")
if [ "$OCC_PUB_COUNT" -ge 1 ] 2>/dev/null; then
    pass "OccGrid publisher aktif ($OCC_PUB_COUNT publisher)"
else
    fail "OccGrid publisher YOK (count: $OCC_PUB_COUNT)"
fi

# =============================================================================
header "DIRECT PIPELINE — OCCUPANCY GRID VERİ ANALİZİ"
# =============================================================================

info "OccupancyGrid mesajı bekleniyor (10 sn)..."
OCC_MSG=$(timeout 10 ros2 topic echo /perception/occupancy_grid_map/map --once 2>/dev/null)

if [ -z "$OCC_MSG" ]; then
    fail "OccupancyGrid mesajı 10 sn içinde GELMEDİ"
else
    pass "OccupancyGrid mesajı ALINDI! 🎉"

    # Frame ID kontrolü
    OCC_FRAME=$(echo "$OCC_MSG" | grep "frame_id:" | head -1 | awk '{print $2}')
    if [ "$OCC_FRAME" = "map" ]; then
        pass "frame_id: map ✓"
    else
        fail "frame_id beklenen 'map', gelen: $OCC_FRAME"
    fi

    # Resolution kontrolü
    OCC_RES=$(echo "$OCC_MSG" | grep "resolution:" | head -1 | awk '{print $2}')
    if [ "$OCC_RES" = "0.5" ]; then
        pass "resolution: 0.5 m/cell ✓"
    else
        warn "resolution: $OCC_RES (beklenen: 0.5)"
    fi

    # Grid boyutu
    OCC_W=$(echo "$OCC_MSG" | grep "width:" | head -1 | awk '{print $2}')
    OCC_H=$(echo "$OCC_MSG" | grep "height:" | head -1 | awk '{print $2}')
    if [ "$OCC_W" = "300" ] && [ "$OCC_H" = "300" ]; then
        pass "Grid boyutu: ${OCC_W}x${OCC_H} (150m x 150m) ✓"
    else
        warn "Grid boyutu: ${OCC_W}x${OCC_H} (beklenen: 300x300)"
    fi

    # Origin kontrolü
    OCC_OX=$(echo "$OCC_MSG" | grep "x:" | head -1 | awk '{print $2}')
    OCC_OY=$(echo "$OCC_MSG" | grep "y:" | head -1 | awk '{print $2}')
    info "Grid origin: ($OCC_OX, $OCC_OY)"

    # Data analizi
    info "OccGrid veri analizi yapılıyor (dolu/boş hücre kontrolü)..."
    OCC_DATA_FILE="/tmp/occ_data_dump.txt"
    # --full-length opsiyonu (bazı ROS2 sürümlerinde --no-arr)
    timeout 5 ros2 topic echo /perception/occupancy_grid_map/map --field data --once --full-length > "$OCC_DATA_FILE" 2>/dev/null || \
    timeout 5 ros2 topic echo /perception/occupancy_grid_map/map --field data --once > "$OCC_DATA_FILE" 2>/dev/null

    if [ -s "$OCC_DATA_FILE" ]; then
        if grep -qE "100|99" "$OCC_DATA_FILE"; then
            pass "Dolu hücreler (99-100) tespit edildi - Engel algılandı! 🛑"
        else
            fail "Dolu hücre (99-100) YOK - Engel algılanamadı!"
        fi

        if grep -q "0" "$OCC_DATA_FILE"; then
            pass "Boş hücreler (0) tespit edildi - Serbest alan algılandı! ⬜"
        else
            fail "Boş hücre (0) YOK - Serbest alan yok!"
        fi

        if grep -q "\-1" "$OCC_DATA_FILE"; then
             pass "Bilinmeyen hücreler (-1) mevcut ⬛"
        else
             warn "Bilinmeyen hücre yok (tüm harita güncellenmiş?)"
        fi
        
        rm -f "$OCC_DATA_FILE"
    else
        fail "OccGrid verisi alınamadı veya boş!"
    fi

    # Engeller doğru konumda mı? (fake scan endpoint doğrulaması)
    # Not: ros2 topic echo varsayılan olarak uzun dizileri kısaltabilir.
    if ros2 topic echo -h 2>&1 | grep -q -- "--full-length"; then
        OCC_DATA_MSG=$(timeout 10 ros2 topic echo /perception/occupancy_grid_map/map --field data --once --full-length 2>/dev/null)
    else
        OCC_DATA_MSG=$(echo "$OCC_MSG" | sed -n '/^data:/,$p')
    fi

    # Çıktı formatı ROS sürümüne göre değişebilir:
    # - YAML liste: "- 50"
    # - Inline liste: "[50, 50, ...]"
    mapfile -t OCC_DATA < <(echo "$OCC_DATA_MSG" | grep -oE -- '-?[0-9]+')
    if [ "${#OCC_DATA[@]}" -eq 0 ]; then
        # Son çare: ilk alınan OCC mesajındaki data bölümünden parse et
        mapfile -t OCC_DATA < <(echo "$OCC_MSG" | sed -n '/^data:/,$p' | grep -oE -- '-?[0-9]+')
    fi

    EXPECTED_CELL_COUNT=$((OCC_W * OCC_H))
    if [ "${#OCC_DATA[@]}" -ne "$EXPECTED_CELL_COUNT" ]; then
        warn "Data hücre sayısı beklenenden farklı: ${#OCC_DATA[@]} (beklenen: $EXPECTED_CELL_COUNT)"
    else
        pass "Data hücre sayısı doğru: ${#OCC_DATA[@]}"

        # Yayınladığımız fake scan içinden seçili ışınlar:
        # idx=2..6 => ranges: [5.0, 4.0, 3.0, 2.0, 1.5]
        ANGLE_MIN="-1.57"
        ANGLE_INCREMENT="0.01"
        BEAM_INDICES=(2 3 4 5 6)
        BEAM_RANGES=(5.0 4.0 3.0 2.0 1.5)

        OCC_HIT_PASS=0
        OCC_HIT_FAIL=0

        for i in "${!BEAM_INDICES[@]}"; do
            beam_idx=${BEAM_INDICES[$i]}
            beam_range=${BEAM_RANGES[$i]}

            beam_angle=$(awk -v amin="$ANGLE_MIN" -v inc="$ANGLE_INCREMENT" -v idx="$beam_idx" 'BEGIN { printf "%.6f", amin + inc * idx }')
            world_x=$(awk -v r="$beam_range" -v a="$beam_angle" 'BEGIN { printf "%.6f", r * cos(a) }')
            world_y=$(awk -v r="$beam_range" -v a="$beam_angle" 'BEGIN { printf "%.6f", r * sin(a) }')

            cell_x=$(awk -v wx="$world_x" -v ox="$OCC_OX" -v res="$OCC_RES" 'BEGIN { print int((wx - ox) / res) }')
            cell_y=$(awk -v wy="$world_y" -v oy="$OCC_OY" -v res="$OCC_RES" 'BEGIN { print int((wy - oy) / res) }')

            if [ "$cell_x" -lt 0 ] || [ "$cell_x" -ge "$OCC_W" ] || [ "$cell_y" -lt 0 ] || [ "$cell_y" -ge "$OCC_H" ]; then
                warn "Beam#$beam_idx (r=$beam_range) grid dışında: cell=($cell_x,$cell_y)"
                ((OCC_HIT_FAIL++))
                continue
            fi

            # Yuvarlama/iz düşüm farklarına tolerans için 3x3 komşulukta kontrol
            hit_found=0
            hit_x=-1
            hit_y=-1
            hit_val=-1

            for dy in -1 0 1; do
                for dx in -1 0 1; do
                    nx=$((cell_x + dx))
                    ny=$((cell_y + dy))
                    if [ "$nx" -lt 0 ] || [ "$nx" -ge "$OCC_W" ] || [ "$ny" -lt 0 ] || [ "$ny" -ge "$OCC_H" ]; then
                        continue
                    fi

                    nidx=$((ny * OCC_W + nx))
                    nval=${OCC_DATA[$nidx]}

                    # Probabilistic map çıktısında engel hücresi 99/100 olabilir.
                    if [ "${nval:-0}" -ge 90 ] 2>/dev/null; then
                        hit_found=1
                        hit_x=$nx
                        hit_y=$ny
                        hit_val=$nval
                        break
                    fi
                done
                if [ "$hit_found" -eq 1 ]; then
                    break
                fi
            done

            if [ "$hit_found" -eq 1 ]; then
                pass "Beam#$beam_idx (r=$beam_range) için occupied bulundu: cell=($hit_x,$hit_y), val=$hit_val"
                ((OCC_HIT_PASS++))
            else
                cidx=$((cell_y * OCC_W + cell_x))
                cval=${OCC_DATA[$cidx]}
                fail "Beam#$beam_idx (r=$beam_range) için occupied yok: beklenen~($cell_x,$cell_y), merkez val=$cval"
                ((OCC_HIT_FAIL++))
            fi
        done

        if [ "$OCC_HIT_FAIL" -eq 0 ] && [ "$OCC_HIT_PASS" -gt 0 ]; then
            pass "Engel konum doğrulaması başarılı ($OCC_HIT_PASS/$OCC_HIT_PASS)"
        else
            fail "Engel konum doğrulaması başarısız (pass=$OCC_HIT_PASS, fail=$OCC_HIT_FAIL)"
        fi
    fi
fi

# Sadece Test 7'yi çalıştırmak için erken çıkış
if [ "$ONLY_TEST7" = "1" ]; then
    info "ONLY_TEST7=1: Test 7 sonrası kalan testler atlanıyor."
    kill $LAUNCH_PID 2>/dev/null
    wait $LAUNCH_PID 2>/dev/null
    kill $SCAN_PID $TF1_PID $TF2_PID 2>/dev/null
    wait $SCAN_PID $TF1_PID $TF2_PID 2>/dev/null

    echo ""
    echo -e "${BOLD}${CYAN}╔═══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}${CYAN}║            📊 TEST SONUÇLARI (ONLY TEST 7)                   ║${NC}"
    echo -e "${BOLD}${CYAN}╚═══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "  ${GREEN}✅ PASS: $pass_count${NC}"
    echo -e "  ${RED}❌ FAIL: $fail_count${NC}"
    echo -e "  ${YELLOW}⚠️  WARN: $warn_count${NC}"
    echo ""

    if [ $fail_count -eq 0 ]; then
        EXIT_CODE=0
    else
        EXIT_CODE=1
    fi
    exit $EXIT_CODE
fi

# =============================================================================
header "DIRECT PIPELINE — NODE BİLGİLERİ"
# =============================================================================

# Node info detayları
NODE_INFO=$(ros2 node info /occupancy_grid_map_node 2>/dev/null)
if [ -n "$NODE_INFO" ]; then
    pass "Node info alındı"

    # Subscriber sayısı
    SUB_COUNT=$(echo "$NODE_INFO" | grep -c "Subscribers:" || echo "0")
    PUB_COUNT=$(echo "$NODE_INFO" | grep -c "Publishers:" || echo "0")
    info "Node detayları:"
    echo "$NODE_INFO" | head -20 | while read line; do echo "    $line"; done
else
    fail "Node info alınamadı"
fi

# Node parametreleri
info "Node parametreleri kontrol ediliyor..."
PARAMS=$(ros2 param list /occupancy_grid_map_node 2>/dev/null)
if [ -n "$PARAMS" ]; then
    pass "Node parametreleri okunabilir"

    # Kritik parametreleri kontrol et
    for param_name in "map_frame" "base_link_frame" "map_length" "map_width" "map_resolution" "updater_type"; do
        PARAM_VAL=$(ros2 param get /occupancy_grid_map_node $param_name 2>/dev/null | tail -1)
        if [ -n "$PARAM_VAL" ]; then
            pass "param $param_name = $PARAM_VAL"
        else
            warn "param $param_name okunamadı"
        fi
    done
else
    fail "Node parametreleri okunamıyor"
fi

# =============================================================================
header "DIRECT PIPELINE — FREKANS TESTİ"
# =============================================================================

info "OccupancyGrid yayın frekansı ölçülüyor (5 sn)..."
OCC_HZ_OUTPUT=$(timeout 5 ros2 topic hz /perception/occupancy_grid_map/map 2>&1)
OCC_HZ=$(echo "$OCC_HZ_OUTPUT" | grep "average rate" | tail -1)

if [ -n "$OCC_HZ" ]; then
    pass "OccGrid frekansı: $OCC_HZ"
    # Hz değerini çıkar
    HZ_NUM=$(echo "$OCC_HZ" | grep -oP '[\d.]+' | head -1)
    if [ -n "$HZ_NUM" ]; then
        # Frekans 1 Hz üstünde mi?
        HZ_INT=${HZ_NUM%%.*}
        if [ "${HZ_INT:-0}" -ge 1 ]; then
            pass "Frekans yeterli (≥1 Hz)"
        else
            warn "Frekans düşük (<1 Hz)"
        fi
    fi
else
    warn "OccGrid frekansı ölçülemedi (veri henüz gelmemiş olabilir)"
fi

# =============================================================================
header "DIRECT PIPELINE — MESAJ SAYISI TESTİ"
# =============================================================================

info "5 saniye boyunca mesaj sayısı ölçülüyor..."
MSG_COUNT_BEFORE=$(ros2 topic echo /perception/occupancy_grid_map/map --field header.stamp.sec 2>/dev/null &
COUNT_PID=$!
sleep 5
kill $COUNT_PID 2>/dev/null
wait $COUNT_PID 2>/dev/null)

# Basit mesaj sayım — 5 sn'de en az 1 mesaj gelmeli
info "5 saniye içinde birden fazla mesaj bekleniyor..."
MSG_FILE="/tmp/occ_msgs_$$.txt"
timeout 5 ros2 topic echo /perception/occupancy_grid_map/map --field header.frame_id > "$MSG_FILE" 2>/dev/null &
MSG_PID=$!
sleep 5
kill $MSG_PID 2>/dev/null
wait $MSG_PID 2>/dev/null

MSG_COUNT=$(wc -l < "$MSG_FILE" 2>/dev/null | tr -d ' ')
rm -f "$MSG_FILE"

if [ "${MSG_COUNT:-0}" -gt 0 ]; then
    pass "5 sn'de $MSG_COUNT satır mesaj alındı"
else
    warn "5 sn'de mesaj sayılamadı"
fi

# =============================================================================
header "DIRECT PIPELINE — İKİNCİ OCCGRID MESAJ KONTROLÜ"
# =============================================================================

info "İkinci bir mesaj daha çekiliyor (tutarlılık testi)..."
OCC_MSG2=$(timeout 8 ros2 topic echo /perception/occupancy_grid_map/map --once 2>/dev/null)
if [ -n "$OCC_MSG2" ]; then
    OCC_FRAME2=$(echo "$OCC_MSG2" | grep "frame_id:" | head -1 | awk '{print $2}')
    OCC_W2=$(echo "$OCC_MSG2" | grep "width:" | head -1 | awk '{print $2}')
    OCC_H2=$(echo "$OCC_MSG2" | grep "height:" | head -1 | awk '{print $2}')

    if [ "$OCC_FRAME2" = "map" ] && [ "$OCC_W2" = "300" ] && [ "$OCC_H2" = "300" ]; then
        pass "İkinci mesaj tutarlı: frame=map, ${OCC_W2}x${OCC_H2} ✓"
    else
        warn "İkinci mesaj farklı: frame=$OCC_FRAME2, ${OCC_W2}x${OCC_H2}"
    fi
else
    warn "İkinci mesaj alınamadı"
fi

# =============================================================================
header "DIRECT PIPELINE — TEMIZLIK"
# =============================================================================

info "Direct pipeline durduruluyor..."
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null
sleep 2

# Node durmuş mu?
if ! ros2 node list 2>/dev/null | grep -q "occupancy_grid_map_node"; then
    pass "Node temiz kapatıldı ✓"
else
    warn "Node hala çalışıyor (zombie olabilir)"
fi

# =============================================================================
header "POINTCLOUD PIPELINE — KONTROL"
# =============================================================================

info "PointCloud pipeline kontrol ediliyor..."
info "(Bu Docker'da PointcloudBasedOccupancyGridMapNode derlenmemiş)"

# laserscan_to_pointcloud node çalışabiliyor mu?
ros2 launch perception_module perception.launch.xml occ_pipeline:=pointcloud 2>&1 &
LAUNCH_PID=$!

info "8 saniye bekleniyor..."
sleep 8

NODE_LIST=$(ros2 node list 2>/dev/null)

# LaserScan → PCL dönüşüm node'u
if echo "$NODE_LIST" | grep -q "laserscan_to_pointcloud"; then
    pass "laserscan_to_pointcloud node'u ÇALIŞIYOR"

    # /points_raw topic'i var mı?
    if ros2 topic list 2>/dev/null | grep -q "/points_raw"; then
        pass "/points_raw topic'i oluştu"

        # PCL mesajı geliyor mu?
        if timeout 5 ros2 topic echo /points_raw --once > /dev/null 2>&1; then
            pass "PointCloud2 mesajları alınıyor 🎯"
        else
            warn "PointCloud2 mesajları gelmiyor (TF sorunu olabilir)"
        fi
    else
        fail "/points_raw topic'i OLUŞMADI"
    fi
else
    warn "laserscan_to_pointcloud node'u başlatılamadı"
fi

# OccGrid oluştu mu? (muhtemelen hayır — plugin yok)
if ros2 topic list 2>/dev/null | grep -q "/perception/occupancy_grid_map/map"; then
    if timeout 5 ros2 topic echo /perception/occupancy_grid_map/map --once > /dev/null 2>&1; then
        pass "PointCloud pipeline OccGrid üretiyor! 🎉 (sürpriz!)"
    else
        warn "OccGrid topic var ama mesaj gelmiyor"
    fi
else
    info "PointCloud pipeline OccGrid üretmiyor (beklenen — plugin yok)"
fi

# Temizle
kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null
sleep 1

# =============================================================================
header "LAUNCH ARGÜMAN TESTLERİ"
# =============================================================================

info "Farklı topic ile test ediliyor..."

# Farklı input topic ile
ros2 topic pub /lidar_scan sensor_msgs/msg/LaserScan "{
  header: {frame_id: 'laser'},
  angle_min: -1.57, angle_max: 1.57, angle_increment: 0.01,
  range_min: 0.1, range_max: 10.0,
  ranges: [3.0, 3.0, 3.0, 3.0, 3.0]
}" --rate 10 &
SCAN2_PID=$!
sleep 1

ros2 launch perception_module perception.launch.xml input_scan_topic:=/lidar_scan 2>&1 &
LAUNCH_PID=$!
sleep 8

if ros2 node list 2>/dev/null | grep -q "occupancy_grid_map_node"; then
    pass "Özel topic (/lidar_scan) ile node başladı ✓"
else
    fail "Özel topic ile node başlatılamadı"
fi

# Node'un doğru topic'e subscribe olduğunu kontrol et
NODE_INFO2=$(ros2 node info /occupancy_grid_map_node 2>/dev/null)
if echo "$NODE_INFO2" | grep -q "lidar_scan"; then
    pass "Node /lidar_scan topic'ine bağlandı ✓"
else
    warn "Node /lidar_scan'a bağlanamamış olabilir"
fi

kill $LAUNCH_PID $SCAN2_PID 2>/dev/null
wait $LAUNCH_PID $SCAN2_PID 2>/dev/null
sleep 1

# =============================================================================
header "HATA DAYANIKLILIĞI TESTLERİ"
# =============================================================================

# 1. Geçersiz pipeline argümanı
info "Geçersiz pipeline argümanı testi..."
INVALID_OUTPUT=$(timeout 5 ros2 launch perception_module perception.launch.xml occ_pipeline:=gecersiz 2>&1)
# Geçersiz pipeline → hiçbir group eşleşmez → node başlamaz ama crash etmemeli
if echo "$INVALID_OUTPUT" | grep -qi "error\|traceback\|exception"; then
    warn "Geçersiz pipeline argümanı hata verdi (crash olmadıysa OK)"
else
    pass "Geçersiz pipeline argümanı — crash yok ✓"
fi

# 2. Scan olmadan launch
info "LaserScan olmadan launch testi..."
ros2 launch perception_module perception.launch.xml 2>&1 &
LAUNCH_PID=$!
sleep 5

if ros2 node list 2>/dev/null | grep -q "occupancy_grid_map_node"; then
    pass "Scan olmadan da node başlıyor (veri bekliyor) ✓"
else
    warn "Scan olmadan node başlamadı"
fi

kill $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null
sleep 1

# =============================================================================
# TF ve scan cleanup
# =============================================================================
kill $SCAN_PID $TF1_PID $TF2_PID 2>/dev/null
wait $SCAN_PID $TF1_PID $TF2_PID 2>/dev/null

# =============================================================================
# SONUÇLAR
# =============================================================================

echo ""
echo -e "${BOLD}${CYAN}╔═══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${CYAN}║                    📊 TEST SONUÇLARI                         ║${NC}"
echo -e "${BOLD}${CYAN}╚═══════════════════════════════════════════════════════════════╝${NC}"
echo ""

TOTAL=$((pass_count + fail_count + warn_count))

echo -e "  ${GREEN}✅ PASS: $pass_count${NC}"
echo -e "  ${RED}❌ FAIL: $fail_count${NC}"
echo -e "  ${YELLOW}⚠️  WARN: $warn_count${NC}"
echo -e "  📝 TOPLAM: $TOTAL"
echo ""

if [ $fail_count -eq 0 ]; then
    echo -e "  ${GREEN}${BOLD}🎉🎉🎉 TÜM TESTLER BAŞARILI! 🎉🎉🎉${NC}"
    echo -e "  ${GREEN}Direct pipeline çalışıyor — LaserScan → OccupancyGrid ✓${NC}"
    EXIT_CODE=0
elif [ $fail_count -le 3 ]; then
    echo -e "  ${YELLOW}⚠️  $fail_count küçük sorun var ama çekirdek çalışıyor${NC}"
    EXIT_CODE=1
else
    echo -e "  ${RED}💥 $fail_count test başarısız — ciddi sorunlar var!${NC}"
    EXIT_CODE=2
fi

echo ""
echo -e "  ${BLUE}Tarih:${NC} $(date '+%Y-%m-%d %H:%M:%S')"
echo ""

exit $EXIT_CODE
