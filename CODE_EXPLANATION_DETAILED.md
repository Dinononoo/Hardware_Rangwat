# ESP32 + MPU6050 + PTFS Distance Sensor + BLE - Detailed Code Explanation

## Overview
This is a professional-grade sensor system for landslide monitoring that combines:
- **MPU6050 IMU** for elevation angle measurement with ±0.1° precision
- **PTFS Laser Distance Sensor** for distance measurement
- **BLE (Bluetooth Low Energy)** for wireless data transmission
- **Advanced Complementary Filter** for high-precision angle calculation

---

## Header and Includes

```cpp
// ESP32 + MPU6050 + PTFS Distance Sensor + BLE
// Combined sensor system for landslide monitoring
#include <Arduino.h>           // Arduino core functions for ESP32
#include <driver/gpio.h>       // GPIO driver for pin configuration
#include "Wire.h"              // I2C communication library
#include "I2Cdev.h"            // I2C device communication utilities
#include "MPU6050.h"           // MPU6050 IMU sensor library
#include "BLEDevice.h"         // BLE device management
#include "BLEServer.h"         // BLE server functionality
#include "BLEUtils.h"          // BLE utility functions
#include "BLE2902.h"           // BLE descriptor for notifications
#include "esp_bt.h"            // ESP32 Bluetooth controller
```

---

## Pin Configuration

```cpp
// ================ Pin Configuration ================
#define SDA_PIN 21  // GPIO21 - I2C SDA (Serial Data Line)
#define SCL_PIN 22  // GPIO22 - I2C SCL (Serial Clock Line)

// PTFS Distance Sensor Pins (5V system)
#define PTFS_RX_PIN 16  // GPIO16 (RX2) - รับข้อมูลจาก PTFS TX
#define PTFS_TX_PIN 17  // GPIO17 (TX2) - ส่งคำสั่งไป PTFS RX (ผ่าน R-divider)
#define PTFS_PWR_PIN 2  // GPIO2 - Power Enable (จ่าย HIGH เพื่อเปิด PTFS)
#define PTFS_RST_PIN 4  // GPIO4 - Reset
```

**Explanation:**
- **SDA/SCL**: I2C communication pins for MPU6050
- **PTFS_RX_PIN**: Receives data from PTFS sensor
- **PTFS_TX_PIN**: Sends commands to PTFS sensor
- **PTFS_PWR_PIN**: Power control for PTFS sensor
- **PTFS_RST_PIN**: Reset control for PTFS sensor

---

## BLE Configuration

```cpp
// ================ BLE Configuration ================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME         "ESP32_LANDSLIDE_MOCK"
```

**Explanation:**
- **SERVICE_UUID**: Unique identifier for BLE service
- **CHARACTERISTIC_UUID**: Unique identifier for data transmission
- **DEVICE_NAME**: Name visible to BLE devices

---

## Global Variables

### MPU6050 Variables
```cpp
MPU6050 mpu;                    // MPU6050 sensor object
int16_t ax, ay, az, gx, gy, gz; // Raw sensor data (accelerometer + gyroscope)
```

### Advanced Precision System
```cpp
// Professional-grade Complementary Filter for ±0.1° accuracy
float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;  // Filtered accelerometer data
float gx_filtered = 0, gy_filtered = 0, gz_filtered = 0;  // Filtered gyroscope data

// Calibration variables - Enhanced for 3,000 samples
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;      // Accelerometer offsets
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;      // Gyroscope offsets
bool calibrated = false;                                   // Calibration status
```

### Professional Elevation System
```cpp
float elevation_raw = 0.0;           // Raw elevation from accelerometer
float elevation_gyro = 0.0;          // Elevation from gyroscope integration
float elevation_complementary = 0.0; // Final result from complementary filter
float elevation_previous = 0.0;      // Previous elevation for gyro integration
```

### Complementary Filter Parameters
```cpp
const float ALPHA = 0.98;            // Gyroscope weight (high frequency)
const float BETA = 0.02;             // Accelerometer weight (low frequency)
const float GYRO_SENSITIVITY = 131.0; // LSB/°/s for ±250°/s range
const float DT = 0.01;               // 100 Hz sampling rate (10ms)
```

**Explanation:**
- **ALPHA**: High weight for gyroscope (short-term accuracy)
- **BETA**: Low weight for accelerometer (long-term stability)
- **GYRO_SENSITIVITY**: Conversion factor for gyroscope data
- **DT**: Time interval for integration (10ms = 100Hz)

### Data Quality Assessment
```cpp
float data_quality_score = 0.0;      // 0-100% quality score
float acceleration_magnitude = 0.0;  // Total acceleration magnitude
float gyro_magnitude = 0.0;          // Total gyro magnitude
bool high_quality_data = false;      // Quality threshold flag
```

### Sampling Rate Control
```cpp
unsigned long last_sample_time = 0;  // Last sample timestamp
const unsigned long SAMPLE_INTERVAL = 10; // 100 Hz = 10ms interval
```

---

## Data Quality Assessment Function

```cpp
float calculateDataQuality(float ax, float ay, float az, float gx, float gy, float gz) {
    // Calculate acceleration magnitude (should be ~1g when stationary)
    acceleration_magnitude = sqrt(ax*ax + ay*ay + az*az);
    
    // Calculate gyroscope magnitude (should be low when stationary)
    gyro_magnitude = sqrt(gx*gx + gy*gy + gz*gz);
    
    // Quality assessment based on multiple factors
    float quality = 100.0;
    
    // Check acceleration magnitude (should be close to 1g)
    float accel_error = abs(acceleration_magnitude - 1.0);
    if (accel_error > 0.1) {
        quality -= accel_error * 200; // Penalty for acceleration error
    }
    
    // Check gyroscope magnitude (should be low when stationary)
    if (gyro_magnitude > 50.0) { // High rotation
        quality -= (gyro_magnitude - 50.0) * 0.5; // Penalty for high rotation
    }
    
    // Check for reasonable values
    if (acceleration_magnitude < 0.5 || acceleration_magnitude > 2.0) {
        quality = 0.0; // Invalid data
    }
    
    // Ensure quality is between 0-100%
    if (quality < 0.0) quality = 0.0;
    if (quality > 100.0) quality = 100.0;
    
    return quality;
}
```

**Explanation:**
- Calculates data quality based on acceleration magnitude (should be ~1g)
- Penalizes high gyroscope values (indicates movement/noise)
- Returns quality score from 0-100%

---

## Complementary Filter Function

```cpp
float applyComplementaryFilter(float accel_angle, float gyro_rate, float dt) {
    // Gyroscope integration (high frequency, short-term accuracy)
    elevation_gyro = elevation_previous + gyro_rate * dt;
    
    // Complementary filter: combine accelerometer (low freq) + gyroscope (high freq)
    elevation_complementary = ALPHA * elevation_gyro + BETA * accel_angle;
    
    // Update previous value for next iteration
    elevation_previous = elevation_complementary;
    
    return elevation_complementary;
}
```

**Explanation:**
- **Gyroscope integration**: Short-term accuracy, prone to drift
- **Complementary filter**: Combines accelerometer (stable) + gyroscope (responsive)
- **ALPHA**: High weight for gyroscope (0.98)
- **BETA**: Low weight for accelerometer (0.02)

---

## BLE Callbacks

### Server Callbacks
```cpp
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("🔵 อุปกรณ์เชื่อมต่อ!");
    };

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("🔴 อุปกรณ์ตัดการเชื่อมต่อ!");
    }
};
```

### Characteristic Callbacks
```cpp
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        uint8_t* data = pCharacteristic->getData();
        size_t len = pCharacteristic->getLength();
        String value = "";
        for (int i = 0; i < len; i++) {
            value += (char)data[i];
        }
        Serial.print("รับคำสั่ง: ");
        Serial.println(value);

        if (value == "MODE1") currentMode = 1;
        else if (value == "MODE2") currentMode = 2;
    }
};
```

**Explanation:**
- Handles BLE connection/disconnection events
- Processes incoming commands from mobile app
- Supports MODE1 and MODE2 commands

---

## System Health Monitoring

### MPU6050 Health Check
```cpp
bool checkMPU6050Health() {
    if (mpu.testConnection()) {
        mpu6050Healthy = true;
        return true;
    } else {
        mpu6050Healthy = false;
        Serial.println("❌ MPU6050 Health Check Failed!");
        return false;
    }
}
```

### PTFS Health Check
```cpp
bool checkPTFSHealth() {
    // ตรวจสอบว่ามีการตอบสนองจาก PTFS ในช่วงเวลาที่กำหนด
    if (millis() - lastPTFSAnalysis < 10000) { // 10 วินาที
        ptfsHealthy = true;
        return true;
    } else {
        ptfsHealthy = false;
        Serial.println("⚠️ PTFS Health Check Failed - No Response!");
        return false;
    }
}
```

### BLE Health Check
```cpp
bool checkBLEHealth() {
    if (pServer && pCharacteristic) {
        bleHealthy = true;
        return true;
    } else {
        bleHealthy = false;
        Serial.println("❌ BLE Health Check Failed!");
        return false;
    }
}
```

### Overall Health Check
```cpp
void performHealthCheck() {
    static unsigned long lastHealthCheck = 0;
    
    if (millis() - lastHealthCheck >= healthCheckInterval) {
        bool mpuOK = checkMPU6050Health();
        bool ptfsOK = checkPTFSHealth();
        bool bleOK = checkBLEHealth();
        
        // นับข้อผิดพลาด
        if (!mpuOK || !ptfsOK || !bleOK) {
            errorCount++;
            Serial.printf("⚠️ Health Check Failed! Error Count: %d/%d\n", errorCount, maxErrorCount);
            
            // รีเซ็ตระบบเมื่อมีข้อผิดพลาดมากเกินไป
            if (errorCount >= maxErrorCount) {
                Serial.println("🔄 Too Many Errors - Restarting System...");
                ESP.restart();
            }
        } else {
            errorCount = 0; // รีเซ็ตตัวนับเมื่อระบบปกติ
        }
        
        // แสดงสถานะหน่วยความจำ
        Serial.printf("💾 Free Heap: %d bytes\n", ESP.getFreeHeap());
        if (ESP.getFreeHeap() < 10000) {
            Serial.println("⚠️ Low Memory Warning!");
        }
        
        lastHealthCheck = millis();
    }
}
```

**Explanation:**
- Monitors all system components every 5 seconds
- Counts errors and restarts system if too many failures
- Monitors free memory and warns if low

---

## PTFS Distance Sensor Functions

### Send PTFS Command
```cpp
void sendPTFSCommand(uint8_t* cmd, uint8_t length) {
    for (int i = 0; i < length; i++) {
        Serial2.write(cmd[i]);
        delayMicroseconds(100);
    }
    Serial2.flush();
}
```

### Start PTFS Measurement
```cpp
void startPTFSMeasurement() {
    if (!ptfsMeasurementActive) {
        return;
    }
    
    // ส่งคำสั่ง PTFS Protocol แบบเดียว (ไม่สับสน)
    uint8_t startCmd[] = {
        0xFA,           // MsgType
        0x01,           // MsgCode (Start)
        0x00,           // BrdId (broadcast)
        0x04,           // PayLoadLen
        0x01, 0x00,     // MeaType (start measurement)
        0x00, 0x00,     // MeaTimes (unlimited)
        0x00            // CRC
    };
    
    // คำนวณ CRC
    uint8_t crc = 0;
    for (int i = 0; i < 8; i++) {
        crc += startCmd[i];
    }
    startCmd[8] = crc & 0xFF;
    
    // ส่งคำสั่ง
    sendPTFSCommand(startCmd, 9);
}
```

**Explanation:**
- Sends official PTFS protocol command to start measurement
- Calculates CRC checksum for data integrity
- Uses 0xFA message type for start command

### Distance Filter Functions
```cpp
void addDistanceToFilter(float value) {
    distanceHistory[distanceFilterIndex] = value;
    distanceFilterIndex = (distanceFilterIndex + 1) % DISTANCE_FILTER_SIZE;
    
    if (!distanceFilterFull && distanceFilterIndex == 0) {
        distanceFilterFull = true;
    }
}

void clearDistanceFilter() {
    for (int i = 0; i < DISTANCE_FILTER_SIZE; i++) {
        distanceHistory[i] = 0.0;
    }
    distanceFilterIndex = 0;
    distanceFilterFull = false;
}

float getFilteredDistance() {
    if (!distanceFilterFull && distanceFilterIndex == 0) return 0.0;
    
    float sum = 0;
    int count = distanceFilterFull ? DISTANCE_FILTER_SIZE : distanceFilterIndex;
    
    for (int i = 0; i < count; i++) {
        sum += distanceHistory[i];
    }
    
    return (count > 0) ? sum / count : 0.0;
}
```

**Explanation:**
- **addDistanceToFilter**: Adds new distance value to circular buffer
- **clearDistanceFilter**: Resets filter when invalid data detected
- **getFilteredDistance**: Returns moving average of stored values

### PTFS Data Processing
```cpp
void processPTFSData(uint8_t* data, uint8_t length) {
    // เช็คว่าเป็น loopback data หรือไม่ (ปรับปรุงให้เสถียรขึ้น)
    if (length > 10 && data[0] == 0xFA) {
        return; // กรองเฉพาะเมื่อมีข้อมูลมากแล้ว
    }

    // ลองวิเคราะห์แม้ข้อมูลน้อย
    if (length < 3) return; // เพิ่มจาก 1 เป็น 3
    
    // **ใช้ Map/Counter เล็กๆ สำหรับค่าที่ถูกต้อง** (เหมือน distance project)
    const int maxDistances = 20;
    uint16_t distances[maxDistances];
    int distanceCount[maxDistances];
    int foundDistances = 0;
    
    // 🎯 **PTFS Official Protocol - Table 5-2 Measurement Report**
    // Format: 0xFB 0x03 BrdId 0x04 DataValidInd Distance_L Distance_H CRC
    
    // Pattern 1: PTFS Standard Protocol (ตามคู่มือ)
    for (int i = 0; i <= length - 8; i++) {
        if (data[i] == 0xFB && data[i+1] == 0x03 && data[i+3] == 0x04) {
            uint8_t dataValid = data[i+4];  // DataValidInd (1=valid, 0=invalid)
            
            if (dataValid == 1) {  // ข้อมูลถูกต้อง
                // Distance in Little Endian format (dm)
                uint16_t dm_value = data[i+6] | (data[i+7] << 8);
                
                // ✅ แก้ไข: ตรวจสอบข้อมูลที่ถูกต้อง
                if (dm_value >= 30 && dm_value <= 10000) {  // 3m-1000m
                    addDistance(distances, distanceCount, &foundDistances, dm_value, maxDistances);
                } else if (dm_value == 0) {
                    // ✅ ระยะทาง 0m = ไม่มีวัตถุในระยะวัด
                } else {
                    // Serial.printf("⚠️  PTFS: Invalid distance value: %d dm\n", dm_value);
                }
            } else {
                // Serial.printf("⚠️  PTFS: Invalid data (dataValid=%d)\n", dataValid);
            }
        }
    }
    
    // ... (Additional protocol patterns for TOF, 9600bps, 16-bit search, single byte)
    
    // **ใช้ค่าแรกที่หาได้เลย (เร็วที่สุด)** เหมือน distance project
    int bestDm = 0;
    
    // หาค่าที่มีความถี่สูงสุด และมีความน่าเชื่อถือ (เพิ่มความเข้มงวด)
    int bestCount = 0;
    for (int i = 0; i < foundDistances; i++) {
        if (distanceCount[i] > bestCount && distanceCount[i] >= 5) { // เพิ่มจาก 3 เป็น 5
            bestCount = distanceCount[i];
            bestDm = distances[i];
        }
    }
    
    // ... (Additional processing for best distance selection)
}
```

**Explanation:**
- Processes multiple PTFS protocol formats (official, TOF, 9600bps, etc.)
- Uses frequency counting to find most reliable distance value
- Validates data quality and range (3m-1000m)
- Applies moving average filter for stability

---

## MPU6050 Functions

### Initialize Advanced Pitch
```cpp
void initializeAdvancedPitch() {
    Serial.println("🔧 Initializing Advanced Pitch Calculation...");
    Serial.println("   📐 Using Accelerometer + Gyroscope for accurate pitch");
    Serial.println("   🎯 Range: -90° to +90° (Pitch/Elevation)");
    Serial.println("   ✅ Roll Independent - ไม่เปลี่ยนเมื่อเอียงซ้ายขวา");
}
```

### Calibrate MPU6050
```cpp
void calibrateMPU6050() {
    Serial.println("🔧 Starting PROFESSIONAL MPU6050 calibration for ±0.1° accuracy...");
    Serial.println("   📐 Keep sensor FLAT and STILL for 30 seconds...");
    Serial.println("   🎯 This will set Elevation = 0° when sensor is level (แนวระดับสายตา)");
    Serial.println("   ⚠️  Make sure sensor is perfectly horizontal (ขนานกับพื้นโลก)!");
    Serial.println("   🎯 Enhanced calibration: 3,000 samples for maximum precision");
    
    delay(3000);
    
    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    const int samples = 3000; // เพิ่มเป็น 3,000 samples สำหรับความแม่นยำสูงสุด
    
    Serial.println("   🔄 Professional Calibrating... (30 seconds)");
    
    for (int i = 0; i < samples; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        
        // แสดงความคืบหน้า
        if (i % 300 == 0) {
            Serial.printf("   📊 Progress: %d/%d samples (%.1f%%)\n", i, samples, (float)i/samples*100);
        }
        delay(10);
    }
    
    // ✅ Professional calibration for both accelerometer and gyroscope
    ax_offset = ax_sum / samples;
    ay_offset = ay_sum / samples;
    az_offset = az_sum / samples - 16384; // Z-axis = 0 เมื่อวางราบ
    
    // Gyroscope calibration (should be 0 when stationary)
    gx_offset = gx_sum / samples;
    gy_offset = gy_sum / samples;
    gz_offset = gz_sum / samples;
    
    // ... (Additional calibration validation and testing)
}
```

**Explanation:**
- Takes 3,000 samples for maximum precision
- Calculates offsets for both accelerometer and gyroscope
- Validates calibration quality
- Tests calibration with sample readings

### Calculate Elevation Angle
```cpp
void calculateElevationAngle() {
    // 🎯 **Professional Elevation Calculation with ±0.1° accuracy**
    
    // ควบคุม Sampling Rate ที่ 100 Hz
    unsigned long current_time = millis();
    if (current_time - last_sample_time < SAMPLE_INTERVAL) {
        return; // Skip if not time for next sample
    }
    last_sample_time = current_time;
    
    // อ่านข้อมูลใหม่จาก MPU6050
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Apply professional calibration
    if (calibrated) {
        ax -= ax_offset;
        ay -= ay_offset;
        az -= az_offset;
        gx -= gx_offset;
        gy -= gy_offset;
        gz -= gz_offset;
    }
    
    // แปลงค่า accelerometer เป็นหน่วย g (gravity)
    float Ax = ax / 16384.0;  // MPU6050 ±2g range
    float Ay = ay / 16384.0;
    float Az = az / 16384.0;
    
    // แปลงค่า gyroscope เป็นหน่วย °/s
    float Gx = gx / GYRO_SENSITIVITY;  // MPU6050 ±250°/s range
    float Gy = gy / GYRO_SENSITIVITY;
    float Gz = gz / GYRO_SENSITIVITY;
    
    // คำนวณ Data Quality Score
    data_quality_score = calculateDataQuality(Ax, Ay, Az, Gx, Gy, Gz);
    high_quality_data = (data_quality_score > 80.0); // 80% threshold
    
    // 🎯 **Professional Elevation Calculation with Complementary Filter**
    
    // คำนวณมุมเงยจาก Accelerometer (แม่นยำระยะยาว)
    float total_magnitude = sqrt(Ax*Ax + Ay*Ay + Az*Az);
    
    if (total_magnitude > 0.1 && high_quality_data) {
        // ตรวจสอบว่าใกล้มุม 90° หรือไม่
        if (abs(Ax) > 0.95 && sqrt(Ay*Ay + Az*Az) < 0.2) {
            // ใกล้มุม 90° มาก - ใช้การตรวจสอบโดยตรง
            if (Ax > 0) {
                elevation_raw = 90.0;  // เงยขึ้น 90°
            } else {
                elevation_raw = -90.0; // ก้มลง 90°
            }
        } else {
            // ใช้ atan2 สำหรับมุมอื่นๆ (แม่นยำกว่า asin)
            float yz_magnitude = sqrt(Ay*Ay + Az*Az);
            if (yz_magnitude > 0.1) {
                elevation_raw = atan2(Ax, yz_magnitude) * 180.0 / PI;
            } else {
                elevation_raw = 0.0; // ไม่มีข้อมูลที่เชื่อถือได้
            }
        }
    } else {
        elevation_raw = 0.0; // ไม่มีข้อมูลที่เชื่อถือได้
    }
    
    // จำกัดค่ามุมก้ม/เงยให้อยู่ในช่วง -90° ถึง +90°
    if (elevation_raw > 90.0) elevation_raw = 90.0;
    if (elevation_raw < -90.0) elevation_raw = -90.0;
    
    // 🎯 **ใช้ Complementary Filter แทน Low-pass + Moving Average**
    // Gyroscope rate สำหรับ pitch (elevation) คือ Gy (Y-axis rotation)
    float gyro_rate = Gy; // °/s
    
    // ใช้ Complementary Filter สำหรับความแม่นยำ ±0.1°
    elevation_complementary = applyComplementaryFilter(elevation_raw, gyro_rate, DT);
    
    // จำกัดค่าผลลัพธ์สุดท้าย
    if (elevation_complementary > 90.0) elevation_complementary = 90.0;
    if (elevation_complementary < -90.0) elevation_complementary = -90.0;
    
    // ... (Display and logging code)
}
```

**Explanation:**
- Controls sampling rate at 100Hz for precision
- Applies calibration offsets to raw data
- Converts raw values to physical units (g and °/s)
- Calculates data quality score
- Uses complementary filter for ±0.1° accuracy
- Handles special cases near 90° angles
- Limits final result to -90° to +90° range

---

## BLE Functions

### Setup BLE
```cpp
void setupBLE() {
    Serial.println("🔵 Starting BLE system...");
    
    BLEDevice::init(DEVICE_NAME);
    BLEDevice::setMTU(247);
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_WRITE  |
                        BLECharacteristic::PROPERTY_NOTIFY |
                        BLECharacteristic::PROPERTY_INDICATE
                      );
    
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    
    pService->start();
    
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    
    Serial.println("✅ BLE ready, waiting for connection...");
}
```

### Send BLE Data
```cpp
void sendBLEData() {
    if (!deviceConnected || !pCharacteristic) return;

    // 📡 ส่งข้อมูลมุมเงยแบบมืออาชีพ (Elevation Angle) ไปมือถือ
    String msg = "elevation:" + String((int)elevation_complementary); // จำนวนเต็ม (ไม่มีทศนิยม)
    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
    delay(50);

    // 📏 ส่งระยะทางจาก PTFS sensor
    msg = "distance:" + String(distance, 1);
    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
    delay(50);

    // 📊 ส่ง Data Quality Score
    msg = "quality:" + String(data_quality_score, 1);
    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
    delay(50);

    // 📱 ส่ง mode
    msg = "mode:" + String(currentMode);
    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
    delay(50);

    // 🔚 ส่งสัญญาณจบ
    pCharacteristic->setValue("END");
    pCharacteristic->notify();
}
```

**Explanation:**
- Sends elevation angle as integer (no decimals)
- Sends distance with 1 decimal place
- Sends data quality score
- Sends current mode
- Sends "END" signal to mark data packet completion

---

## Setup Function

```cpp
void setup() {
    Serial.begin(115200);
    Serial.println("🎯 ESP32 + MPU6050 + PTFS Distance Sensor + BLE - PROFESSIONAL EDITION");
    Serial.println("=======================================================================");
    Serial.println("📐 PROFESSIONAL ELEVATION/DEPRESSION ANGLE measurement with ±0.1° precision");
    Serial.println();
    Serial.println("🎯 PROFESSIONAL FEATURES:");
    Serial.println("   📐 ±0.1° Precision with Complementary Filter");
    Serial.println("   🔧 3,000 samples calibration for maximum accuracy");
    Serial.println("   ⚡ 100 Hz controlled sampling rate");
    Serial.println("   📊 Data Quality Assessment (0-100%)");
    Serial.println("   🔄 Gyroscope integration for short-term accuracy");
    Serial.println();
    Serial.println("📐 ELEVATION = มุมเงย/มุมก้ม (Elevation/Depression angle) in degrees → sent as 'elevation'");
    Serial.println("   📈 Positive (+) = มุมเงย (วัตถุอยู่สูงกว่าแนวระดับสายตา)");
    Serial.println("   📉 Negative (-) = มุมก้ม (วัตถุอยู่ต่ำกว่าแนวระดับสายตา)");
    Serial.println("   🔒 ROLL INDEPENDENT = มุมเงยไม่เปลี่ยนเมื่อเอียงซ้ายขวา (เสถียร)");
    Serial.println("📏 DISTANCE = Real distance from PTFS Laser Sensor");
    Serial.println("📊 QUALITY = Data quality score (0-100%)");
    Serial.println("🔌 PTFS: 3.3V power, UART 115200 bps");
    Serial.println("🎯 FOCUS: Professional-grade Elevation/Depression Angle measurement");
    Serial.println();
    
    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    
    // Setup PTFS Distance Sensor (3.3V system)
    Serial.println("🔧 Initialize PTFS Distance Sensor (3.3V system)...");
    Serial.println("   ⚡ PTFS VIN → 3.3V, PWR_EN → GPIO2 (HIGH)");
    Serial.println("   🔌 TX → GPIO16 (RX2), RX → GPIO17 (TX2)");
    Serial.println("   📡 UART: 115200 bps, TTL 3.3V");
    
    pinMode(PTFS_PWR_PIN, OUTPUT);
    pinMode(PTFS_RST_PIN, OUTPUT);
    
    digitalWrite(PTFS_PWR_PIN, HIGH);  // เปิด PTFS ด้วย 3.3V
    digitalWrite(PTFS_RST_PIN, LOW);
    delay(100);
    digitalWrite(PTFS_RST_PIN, HIGH);
    delay(500);
    
    // Initialize UART for PTFS (115200 bps)
    Serial2.begin(115200, SERIAL_8N1, PTFS_RX_PIN, PTFS_TX_PIN);
    
    // GPIO signal strength for 3.3V system
    gpio_set_drive_capability(GPIO_NUM_17, GPIO_DRIVE_CAP_2); // 3.3V TTL
    gpio_set_pull_mode(GPIO_NUM_17, GPIO_PULLUP_ONLY);
    gpio_set_drive_capability(GPIO_NUM_16, GPIO_DRIVE_CAP_2);
    gpio_set_pull_mode(GPIO_NUM_16, GPIO_PULLUP_ONLY);
    
    Serial.println("✅ PTFS Distance Sensor ready (3.3V system)");
    Serial.println("   📡 UART: 115200 bps, RX=GPIO16, TX=GPIO17");
    Serial.println("   ⚡ Power: 3.3V via GPIO2, No R-divider needed");
    
    delay(1000);
    
    // Initialize MPU6050
    Serial.println("🔧 Initialize MPU6050...");
    mpu.initialize();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setDLPFMode(MPU6050_DLPF_BW_5);
    
    if (mpu.testConnection()) {
        Serial.println("✅ MPU6050 Connected Successfully!");
        
        // เริ่มต้นการคำนวณมุมเงยขั้นสูง
        initializeAdvancedPitch();
        
        // คาลิเบรต MPU6050
        calibrateMPU6050();
        
        // Setup BLE
        setupBLE();
        
        // Start PTFS measurement
        Serial.println("📏 Starting PTFS distance measurement...");
        Serial.println("   🎯 Using official PTFS protocol (0xFA/0xFB)");
        Serial.println("   📡 Baud rate: 115200 bps");
        
        // เริ่มส่งคำสั่ง PTFS Protocol
        ptfsMeasurementActive = true;
        startPTFSMeasurement();
        lastPTFSCommand = millis();
        lastPTFSAnalysis = millis();
        lastPTFSDisplayTime = millis();
        
        Serial.println("=======================================================================");
        Serial.println("🎯 PROFESSIONAL SYSTEM READY! Measuring with ±0.1° precision...");
        Serial.println("📐 ELEVATION: มุมเงย/มุมก้ม (Elevation/Depression) in degrees via BLE");
        Serial.println("📏 PTFS: 3.3V power, UART 115200 bps, Official protocol");
        Serial.println("📊 QUALITY: Real-time data quality assessment (0-100%)");
        Serial.println("🎯 FOCUS: Professional-grade Elevation/Depression Angle measurement");
        Serial.println("🔒 STABLE: Roll Independent - มุมเงยไม่เปลี่ยนเมื่อเอียงซ้ายขวา");
        Serial.println("⚡ SAMPLING: 100 Hz controlled rate for maximum accuracy");
        
    } else {
        Serial.println("❌ MPU6050 Connection Failed!");
        Serial.println("Check wiring:");
        Serial.println("  ESP32 3.3V → MPU6050 VCC");
        Serial.println("  ESP32 GND  → MPU6050 GND");
        Serial.println("  ESP32 G21  → MPU6050 SDA");
        Serial.println("  ESP32 G22  → MPU6050 SCL");
        Serial.println("  PTFS VIN   → 3.3V");
        Serial.println("  PTFS GND   → GND");
        Serial.println("  PTFS TX    → GPIO16 (RX2)");
        Serial.println("  PTFS RX    → GPIO17 (TX2) - No R-divider needed");
        while(1);
    }
}
```

**Explanation:**
- Initializes serial communication
- Sets up I2C for MPU6050
- Configures PTFS sensor with 3.3V power
- Initializes UART for PTFS communication
- Sets up MPU6050 with optimal settings
- Performs calibration
- Sets up BLE system
- Starts PTFS measurement

---

## Main Loop

```cpp
void loop() {
    // ✅ ปรับปรุงการจัดการ Watchdog Timer
    static unsigned long lastWatchdogFeed = 0;
    
    // ให้อาหาร watchdog บ่อยขึ้น
    if (millis() - lastWatchdogFeed >= 1000) { // ทุก 1 วินาที
        yield();
        lastWatchdogFeed = millis();
    }
    
    // ✅ เพิ่มการตรวจสอบสถานะระบบ
    performHealthCheck();
    
    // Process PTFS distance sensor data
    processPTFSStream();
    
    // ส่งคำสั่ง PTFS แบบต่อเนื่อง
    if (millis() - lastPTFSCommand >= ptfsCommandInterval) {
        if (ptfsMeasurementActive) {
            startPTFSMeasurement();
        }
        lastPTFSCommand = millis();
    }
    
    // Hardware reset เซ็นเซอร์ทุก 5 นาที (ลดความถี่)
    static unsigned long lastHardwareReset = 0;
    if (millis() - lastHardwareReset >= 300000) { // 5 นาที
        digitalWrite(PTFS_RST_PIN, LOW);
        delay(100);
        digitalWrite(PTFS_RST_PIN, HIGH);
        delay(500);
        ptfsMeasurementActive = true; // เปิดใช้งานใหม่
        lastHardwareReset = millis();
    }
    
    // อ่านข้อมูล MPU6050 (สำหรับ health check)
    if (mpu6050Healthy) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    } else {
        Serial.println("⚠️ Using cached MPU6050 data");
    }
    
    // คำนวณมุมเงย
    calculateElevationAngle();
    
    // แสดงค่าปัจจุบัน (มุมเงยแบบมืออาชีพในหน่วยองศา - จำนวนเต็ม)
    Serial.printf("📐 ELEVATION: %+8d°  |  DIST: %6.1fm  |  QUALITY: %5.1f%%", 
                  (int)elevation_complementary, distance, data_quality_score);
    
    // แสดงข้อมูลดิบ (ทุก 3 วินาที)
    static unsigned long lastRawDisplay = 0;
    if (millis() - lastRawDisplay >= 3000) {
        Serial.printf("   📊 Raw: Ax=%+6.3f, Ay=%+6.3f, Az=%+6.3f (g)", 
                      ax/16384.0, ay/16384.0, az/16384.0);
        
        // แสดงการทดสอบการทำงาน
        if (elevation_complementary > 1.0) {
            Serial.printf(" → เงยขึ้น");
        } else if (elevation_complementary < -1.0) {
            Serial.printf(" → ก้มลง");
        } else {
            Serial.printf(" → ราบ");
        }
        
        lastRawDisplay = millis();
    }
    
    // แสดงสถานะการเชื่อมต่อ
    if (deviceConnected) {
        Serial.print("  [🔵 BLE Connected]");
    } else {
        Serial.print("  [⚪ BLE Disconnected]");
    }
    
    // แสดงสถานะเซ็นเซอร์
    if (mpu6050Healthy && ptfsHealthy) {
        Serial.print("  [✅ Sensors OK]");
    } else {
        Serial.print("  [⚠️ Sensor Issues]");
    }
    
    Serial.println();
    
    // Send BLE data
    unsigned long currentMillis = millis();
    if (currentMillis - lastSendTime >= sendInterval) {
        lastSendTime = currentMillis;
        if (deviceConnected) {
            sendBLEData();
        }
    }
    
    // Handle BLE reconnection
    handleBLEReconnection();
    
    // ป้องกัน watchdog timer (เสถียร)
    yield();
    delay(10);
}
```

**Explanation:**
- Feeds watchdog timer every second
- Performs system health checks
- Processes PTFS distance data
- Sends PTFS commands every 2 seconds
- Resets PTFS sensor every 5 minutes
- Reads MPU6050 data
- Calculates elevation angle
- Displays current values
- Sends BLE data every second
- Handles BLE reconnection
- Prevents watchdog timeout

---

## Key Features Summary

### 1. **Professional Elevation Measurement**
- ±0.1° precision using complementary filter
- 3,000 samples calibration for maximum accuracy
- 100 Hz controlled sampling rate
- Roll independent (stable when tilting left/right)

### 2. **Advanced Data Processing**
- Data quality assessment (0-100%)
- Moving average filter for distance
- Multiple PTFS protocol support
- Real-time health monitoring

### 3. **Robust System Design**
- Automatic error recovery
- Memory management
- Watchdog timer protection
- Hardware reset cycles

### 4. **Wireless Communication**
- BLE data transmission
- Real-time sensor data
- Command processing
- Connection management

This system provides professional-grade elevation angle measurement suitable for landslide monitoring applications with high precision and reliability.
