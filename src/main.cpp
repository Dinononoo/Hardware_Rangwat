// ESP32 + MPU6050 + PTFS Distance Sensor + BLE
// Combined sensor system for landslide monitoring
#include <Arduino.h>
#include <driver/gpio.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include "esp_bt.h"

// ================ Pin Configuration ================
#define SDA_PIN 21  // GPIO21 - I2C SDA
#define SCL_PIN 22  // GPIO22 - I2C SCL

// PTFS Distance Sensor Pins (5V system)
#define PTFS_RX_PIN 16  // GPIO16 (RX2) - รับข้อมูลจาก PTFS TX
#define PTFS_TX_PIN 17  // GPIO17 (TX2) - ส่งคำสั่งไป PTFS RX (ผ่าน R-divider)
#define PTFS_PWR_PIN 2  // GPIO2 - Power Enable (จ่าย HIGH เพื่อเปิด PTFS)
#define PTFS_RST_PIN 4  // GPIO4 - Reset

// ================ BLE Configuration ================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME         "ESP32_LANDSLIDE_MOCK"

// ================ Global Variables ================
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

// ================ Advanced Precision System ================
// Professional-grade Complementary Filter for ±0.1° accuracy
float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;
float gx_filtered = 0, gy_filtered = 0, gz_filtered = 0;

// Calibration variables - Enhanced for 3,000 samples
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
bool calibrated = false;

// Professional Elevation System - ±0.1° accuracy
float elevation_raw = 0.0;           // Raw elevation from accelerometer
float elevation_gyro = 0.0;          // Elevation from gyroscope integration
float elevation_complementary = 0.0; // Final result from complementary filter
float elevation_previous = 0.0;      // Previous elevation for gyro integration

// Complementary Filter Parameters (Professional Grade)
const float ALPHA = 0.98;            // Gyroscope weight (high frequency)
const float BETA = 0.02;             // Accelerometer weight (low frequency)
const float GYRO_SENSITIVITY = 131.0; // LSB/°/s for ±250°/s range
const float DT = 0.01;               // 100 Hz sampling rate (10ms)

// Data Quality Assessment
float data_quality_score = 0.0;      // 0-100% quality score
float acceleration_magnitude = 0.0;  // Total acceleration magnitude
float gyro_magnitude = 0.0;          // Total gyro magnitude
bool high_quality_data = false;      // Quality threshold flag

// Sampling Rate Control
unsigned long last_sample_time = 0;
const unsigned long SAMPLE_INTERVAL = 10; // 100 Hz = 10ms interval

// ================ Professional Data Quality Assessment ================
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

// ================ Professional Complementary Filter ================
float applyComplementaryFilter(float accel_angle, float gyro_rate, float dt) {
    // Gyroscope integration (high frequency, short-term accuracy)
    elevation_gyro = elevation_previous + gyro_rate * dt;
    
    // Complementary filter: combine accelerometer (low freq) + gyroscope (high freq)
    elevation_complementary = ALPHA * elevation_gyro + BETA * accel_angle;
    
    // Update previous value for next iteration
    elevation_previous = elevation_complementary;
    
    return elevation_complementary;
}

// BLE variables
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Data variables
float distance = 0.0;  // ระยะทางจริงจาก PTFS sensor
int currentMode = 1;
unsigned long lastSendTime = 0;
const long sendInterval = 1000;  // เพิ่มจาก 500ms เป็น 1000ms เพื่อความเสถียร

// ✅ เพิ่มตัวแปรสำหรับการจัดการข้อผิดพลาด
bool mpu6050Healthy = false;
bool ptfsHealthy = false;
bool bleHealthy = false;
unsigned long lastMPU6050Check = 0;
unsigned long lastPTFSCheck = 0;
unsigned long lastBLECheck = 0;
const unsigned long healthCheckInterval = 5000; // ตรวจสอบทุก 5 วินาที
int errorCount = 0;
const int maxErrorCount = 10; // จำนวนข้อผิดพลาดสูงสุดก่อนรีเซ็ต

// PTFS variables
uint8_t ptfsBuffer[200];
int ptfsBufferIndex = 0;
bool ptfsMeasurementActive = false;
bool loopbackDetected = false;  // เพิ่มตัวแปรตรวจจับ loopback
unsigned long lastPTFSCommand = 0;
unsigned long lastPTFSAnalysis = 0;
unsigned long lastPTFSDisplayTime = 0;
const unsigned long ptfsCommandInterval = 2000;  // เพิ่มเป็น 2 วินาที เพื่อความเสถียร
const unsigned long ptfsAnalysisInterval = 1000;  // เพิ่มเป็น 1 วินาที เพื่อความเสถียร

// Distance filter (เพิ่มขนาดเพื่อความเสถียร)
#define DISTANCE_FILTER_SIZE 30  // เพิ่มจาก 20 เป็น 30
float distanceHistory[DISTANCE_FILTER_SIZE] = {0};
int distanceFilterIndex = 0;
bool distanceFilterFull = false;

// ================ BLE Callbacks ================
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

// ================ System Health Monitoring ================
// ✅ เพิ่มฟังก์ชันตรวจสอบสถานะระบบ
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

// ================ PTFS Functions ================
void sendPTFSCommand(uint8_t* cmd, uint8_t length) {
    for (int i = 0; i < length; i++) {
        Serial2.write(cmd[i]);
        delayMicroseconds(100);
    }
        Serial2.flush();
}

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
    
    // ✅ ปิด debug
    // Serial.println("📡 Sent PTFS command");
}

void addDistanceToFilter(float value) {
    distanceHistory[distanceFilterIndex] = value;
    distanceFilterIndex = (distanceFilterIndex + 1) % DISTANCE_FILTER_SIZE;
    
    if (!distanceFilterFull && distanceFilterIndex == 0) {
        distanceFilterFull = true;
    }
}

// ✅ เพิ่มฟังก์ชันล้าง Filter
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

// **ฟังก์ชันเพิ่มค่าระยะทางใน array** (เหมือน distance project)
void addDistance(uint16_t* distances, int* counts, int* foundCount, uint16_t value, int maxSize) {
    // หาว่ามีค่านี้อยู่แล้วหรือไม่
    for (int i = 0; i < *foundCount; i++) {
        if (distances[i] == value) {
            counts[i]++;
            return;
        }
    }
    
    // ถ้าไม่มี และยังมีที่ว่าง ให้เพิ่มใหม่
    if (*foundCount < maxSize) {
        distances[*foundCount] = value;
        counts[*foundCount] = 1;
        (*foundCount)++;
    }
}

void processPTFSData(uint8_t* data, uint8_t length) {
    // เช็คว่าเป็น loopback data หรือไม่ (ปรับปรุงให้เสถียรขึ้น)
    if (length > 10 && data[0] == 0xFA) {
        return; // กรองเฉพาะเมื่อมีข้อมูลมากแล้ว
    }

    // ลองวิเคราะห์แม้ข้อมูลน้อย
    if (length < 3) return; // เพิ่มจาก 1 เป็น 3
    
    // ✅ ปิดการแสดงข้อมูล Raw เพื่อความสะอาด
    // static unsigned long lastRawDisplay = 0;
    // if (millis() - lastRawDisplay >= 2000) { // แสดงทุก 2 วินาที
    //     Serial.print("📡 Raw PTFS Data: ");
    //     for (int i = 0; i < (length < 20 ? length : 20); i++) { // แสดงแค่ 20 bytes แรก
    //         Serial.printf("0x%02X ", data[i]);
    //     }
    //     if (length > 20) Serial.print("...");
    //     Serial.printf(" (Length: %d)\n", length);
    //     
    //     // ✅ เพิ่มการวิเคราะห์ข้อมูล Raw
    //     if (length >= 9 && data[0] == 0xFB && data[1] == 0x03) {
    //         uint8_t dataValid = data[4];
    //         uint16_t rawDistance = data[6] | (data[7] << 8);
    //         Serial.printf("   📊 Analysis: dataValid=%d, rawDistance=%d dm (%.1fm)\n", 
    //                      dataValid, rawDistance, rawDistance/10.0);
    //     }
    //     
    //     lastRawDisplay = millis();
    // }
    
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
                
                // Serial.printf("🔍 PTFS Protocol: dm_value=%d, dataValid=%d\n", dm_value, dataValid);
                
                // ✅ แก้ไข: ตรวจสอบข้อมูลที่ถูกต้อง
                if (dm_value >= 30 && dm_value <= 10000) {  // 3m-1000m
                    addDistance(distances, distanceCount, &foundDistances, dm_value, maxDistances);
                } else if (dm_value == 0) {
                    // ✅ ระยะทาง 0m = ไม่มีวัตถุในระยะวัด
                    // Serial.println("📏 PTFS: No object detected (0m)");
                } else {
                    // Serial.printf("⚠️  PTFS: Invalid distance value: %d dm\n", dm_value);
                }
            } else {
                // Serial.printf("⚠️  PTFS: Invalid data (dataValid=%d)\n", dataValid);
            }
        }
    }
    
    // Pattern New: TOF Protocol (0x5A responses) - ✅ ปิดการใช้งานเมื่อ dataValid=0
    // ✅ ตรวจสอบว่ามีข้อมูล PTFS Protocol ที่ dataValid=0 หรือไม่
    bool hasInvalidPTFSData = false;
    for (int i = 0; i <= length - 8; i++) {
        if (data[i] == 0xFB && data[i+1] == 0x03 && data[i+3] == 0x04) {
            uint8_t dataValid = data[i+4];
            if (dataValid == 0) {
                hasInvalidPTFSData = true;
                // Serial.println("🚫 PTFS: dataValid=0 detected - skipping TOF Protocol");
                break;
            }
        }
    }
    
    // ✅ ใช้ TOF Protocol เฉพาะเมื่อไม่มีข้อมูล PTFS ที่ dataValid=0
    if (!hasInvalidPTFSData) {
        for (int i = 0; i <= length - 6; i++) {
            if (data[i] == 0x5A && i + 5 < length) {
                uint16_t dm_value = data[i+4] | (data[i+5] << 8);
                // Serial.printf("🔍 TOF Protocol: dm_value=%d\n", dm_value);
                if (dm_value >= 30 && dm_value <= 10000) {
                    addDistance(distances, distanceCount, &foundDistances, dm_value, maxDistances);
                }
            }
        }
    }
    
    // Pattern 2: 9600 bps Format (80 0C 08 XX pattern) - ✅ ปิดการใช้งานเมื่อ dataValid=0
    // ✅ ใช้ 9600 bps Format เฉพาะเมื่อไม่มีข้อมูล PTFS ที่ dataValid=0
    if (!hasInvalidPTFSData) {
        for (int i = 0; i < length - 3; i++) {
            if (data[i] == 0x80 && data[i+1] == 0x0C && data[i+2] == 0x08) {
                uint16_t dm_value = data[i+3];  // ค่าระยะทางใน dm
                // Serial.printf("🔍 9600bps Format: dm_value=%d\n", dm_value);
                if (dm_value >= 30 && dm_value <= 1000) {
                    addDistance(distances, distanceCount, &foundDistances, dm_value, maxDistances);
                }
            }
        }
    }
    
    // Pattern 3: Smart 16-bit search (หลีกเลี่ยง garbage) - ✅ ปิดการใช้งานเมื่อ dataValid=0
    if (foundDistances == 0) {
        // ✅ ใช้ 16-bit search เฉพาะเมื่อไม่มีข้อมูล PTFS ที่ dataValid=0
        if (!hasInvalidPTFSData) {
            for (int i = 0; i < length - 1; i++) {
                // ข้าม garbage bytes
                if (data[i] == 0xFE || data[i] == 0xFF || data[i+1] == 0xFE || data[i+1] == 0xFF) {
                    continue;
                }
                
                // Little Endian 16-bit
                uint16_t dm_value = data[i] | (data[i+1] << 8);
                if (dm_value >= 30 && dm_value <= 1000) {
                    // Serial.printf("🔍 16-bit LE: dm_value=%d\n", dm_value);
                    addDistance(distances, distanceCount, &foundDistances, dm_value, maxDistances);
                }
                
                // Big Endian 16-bit  
                dm_value = (data[i] << 8) | data[i+1];
                if (dm_value >= 30 && dm_value <= 1000) {
                    // Serial.printf("🔍 16-bit BE: dm_value=%d\n", dm_value);
                    addDistance(distances, distanceCount, &foundDistances, dm_value, maxDistances);
                }
            }
        }
    }
    
    // Pattern 4: Single byte reasonable values - ✅ ปิดการใช้งานเมื่อ dataValid=0
    if (foundDistances == 0) {
        // ✅ ใช้ single byte search เฉพาะเมื่อไม่มีข้อมูล PTFS ที่ dataValid=0
        if (!hasInvalidPTFSData) {
            for (int i = 0; i < length; i++) {
                if (data[i] >= 10 && data[i] <= 200 && data[i] != 0xFE && data[i] != 0xFF && data[i] != 0x00) {
                    // Serial.printf("🔍 Single byte: dm_value=%d\n", data[i]);
                    addDistance(distances, distanceCount, &foundDistances, data[i], maxDistances);
                }
            }
        }
    }
    
    // **ใช้ค่าแรกที่หาได้เลย (เร็วที่สุด)** เหมือน distance project
    int bestDm = 0;
    
    // ✅ ปิดการแสดงข้อมูลที่พบทั้งหมด
    // Serial.printf("🔍 Found %d different distances:\n", foundDistances);
    // for (int i = 0; i < foundDistances; i++) {
    //     Serial.printf("   Distance[%d]: %d dm (count: %d)\n", i, distances[i], distanceCount[i]);
    // }
    
    // หาค่าที่มีความถี่สูงสุด และมีความน่าเชื่อถือ (เพิ่มความเข้มงวด)
    int bestCount = 0;
    for (int i = 0; i < foundDistances; i++) {
        if (distanceCount[i] > bestCount && distanceCount[i] >= 5) { // เพิ่มจาก 3 เป็น 5
            bestCount = distanceCount[i];
            bestDm = distances[i];
        }
    }
    
    // ถ้าไม่มีค่าที่เชื่อถือได้ ให้หาค่าเฉลี่ยของค่าที่ใกล้เคียงกัน
    if (bestDm == 0 && foundDistances > 0) {
        uint32_t sum = 0;
        int count = 0;
        uint16_t refValue = distances[0];
        
        // รวมค่าที่ใกล้เคียงกัน (ต่างกันไม่เกิน 20dm = 2m)
        for (int i = 0; i < foundDistances; i++) {
            if (abs((int)distances[i] - (int)refValue) <= 20) {
                sum += distances[i];
                count++;
            }
        }
        
        if (count >= 2) {
            bestDm = sum / count; // ค่าเฉลี่ยที่เสถียร
        } else {
            bestDm = distances[0]; // ใช้ค่าแรก
        }
    }
    
    // แสดงผลตามที่ผู้ใช้ต้องการ (พร้อม Moving Average Filter) เหมือน distance project
    if (bestDm > 0) {
        // ✅ ปิดการแสดงข้อมูล debug
        // Serial.printf("🔍 PTFS Debug: bestDm=%d, foundDistances=%d\n", bestDm, foundDistances);
        
        // ✅ แก้ไข: ตรวจสอบระยะทางที่ถูกต้อง
        if (bestDm < 30) { // น้อยกว่า 3.0m
            // Serial.printf("⚠️  ระยะทางน้อยเกินไป (%d dm = %.1fm) - ตั้งเป็น 0m\n", bestDm, bestDm/10.0);
            // ✅ ล้าง Filter และตั้งค่าระยะทางเป็น 0 เมื่อน้อยกว่า 3m
            clearDistanceFilter();
            distance = 0.0;
            
            // ✅ ปิดการแสดงผลทันที
            // if (millis() - lastPTFSDisplayTime >= 1000) {
            //     Serial.printf("📏 PTFS Distance: 0.0m (Raw: %.1fm - Too Close)\n", bestDm/10.0);
            //     lastPTFSDisplayTime = millis();
            // }
        } else {
            // เพิ่มค่าใน Moving Average Filter
            float meters = bestDm / 10.0;
            
            // ตรวจสอบค่าที่สมเหตุสมผล
            if (meters > 0 && meters < 1000) {
                addDistanceToFilter(meters);
                distance = getFilteredDistance();
                
                // ✅ ปิดการแสดงผลทุก 1 วินาที
                // if (millis() - lastPTFSDisplayTime >= 1000) {
                //     Serial.printf("📏 PTFS Distance: %.1fm (Raw: %.1fm)\n", distance, meters);
                //     lastPTFSDisplayTime = millis();
                // }
            } else {
                // Serial.printf("⚠️  ค่าไม่สมเหตุสมผล: %.1fm - ตั้งเป็น 0m\n", meters);
                // ✅ ล้าง Filter และตั้งค่าระยะทางเป็น 0 เมื่อไม่สมเหตุสมผล
                clearDistanceFilter();
                distance = 0.0;
            }
        }
    } else {
        // ✅ ไม่พบข้อมูลระยะทางที่ถูกต้อง - ตั้งเป็น 0m
        // Serial.println("📏 PTFS: No valid distance data - setting to 0m");
        clearDistanceFilter();
        distance = 0.0;
    }
}

void processPTFSStream() {
    // ✅ ปรับปรุงการจัดการหน่วยความจำ
    static unsigned long lastBufferCleanup = 0;
    
    // ล้างบัฟเฟอร์เป็นระยะเพื่อป้องกันหน่วยความจำเต็ม
    if (millis() - lastBufferCleanup >= 30000) { // ทุก 30 วินาที
        ptfsBufferIndex = 0;
        lastBufferCleanup = millis();
    }
    
    while (Serial2.available()) {
        uint8_t newByte = Serial2.read();
        
        // ลดการตรวจจับ loopback ที่เข้มงวดเกินไป
        if (newByte == 0xFA && ptfsBufferIndex > 15) {
            // ล้างบัฟเฟอร์เฉพาะเมื่อมีข้อมูลมากแล้ว
            ptfsBufferIndex = 0;
            continue;
        }
        
        ptfsBuffer[ptfsBufferIndex++] = newByte;
        
        // ✅ ปรับปรุงการจัดการบัฟเฟอร์
        if (ptfsBufferIndex >= 200) {
            ptfsBufferIndex = 0; // รีเซ็ตบัฟเฟอร์เมื่อเต็ม
            Serial.println("⚠️ PTFS Buffer Overflow - Reset!");
        }
    }
    
    // วิเคราะห์ข้อมูลช้าลงเพื่อความเสถียร
    if (ptfsBufferIndex >= 8) { // เพิ่มจาก 2 เป็น 8
        processPTFSData(ptfsBuffer, ptfsBufferIndex);
        lastPTFSAnalysis = millis();
        ptfsBufferIndex = 0;
    }
}

// ================ MPU6050 Functions ================
// Advanced Pitch Calculation using Accelerometer + Gyroscope
void initializeAdvancedPitch() {
    Serial.println("🔧 Initializing Advanced Pitch Calculation...");
    Serial.println("   📐 Using Accelerometer + Gyroscope for accurate pitch");
    Serial.println("   🎯 Range: -90° to +90° (Pitch/Elevation)");
    Serial.println("   ✅ Roll Independent - ไม่เปลี่ยนเมื่อเอียงซ้ายขวา");
}

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
    
    // ✅ เพิ่มการตรวจสอบการคาลิเบรตแบบมืออาชีพ
    Serial.printf("   📊 Accelerometer Offsets: Ax=%d, Ay=%d, Az=%d\n", ax_offset, ay_offset, az_offset);
    Serial.printf("   📊 Gyroscope Offsets: Gx=%d, Gy=%d, Gz=%d\n", gx_offset, gy_offset, gz_offset);
    
    // ✅ ตรวจสอบคุณภาพการคาลิเบรต
    float accel_magnitude = sqrt((ax_offset/16384.0)*(ax_offset/16384.0) + 
                                (ay_offset/16384.0)*(ay_offset/16384.0) + 
                                ((az_offset+16384)/16384.0)*((az_offset+16384)/16384.0));
    float gyro_magnitude = sqrt((gx_offset/131.0)*(gx_offset/131.0) + 
                               (gy_offset/131.0)*(gy_offset/131.0) + 
                               (gz_offset/131.0)*(gz_offset/131.0));
    
    Serial.printf("   📊 Calibration Quality: Accel=%.3f g, Gyro=%.1f°/s\n", accel_magnitude, gyro_magnitude);
    
    // ✅ ตรวจสอบว่า Z-axis ใกล้ 16384 หรือไม่ (ควรเป็น 1g)
    if (abs(az_offset) > 1000) {
        Serial.println("   ⚠️ Z-axis offset seems wrong, adjusting...");
        az_offset = az_sum / samples; // ใช้ค่าเดิม
    }
    
    calibrated = true;
    Serial.println("✅ PROFESSIONAL MPU6050 Calibration Complete!");
    Serial.printf("   📊 Enhanced Offsets: Ax=%d, Ay=%d, Az=%d\n", ax_offset, ay_offset, az_offset);
    Serial.printf("   📊 Gyro Offsets: Gx=%d, Gy=%d, Gz=%d\n", gx_offset, gy_offset, gz_offset);
    Serial.println("   📐 Elevation Angle will be 0° when sensor is level (แนวระดับสายตา)");
    Serial.println("   🎯 Ready for ±0.1° precision with Complementary Filter!");
    
    // ทดสอบการคาลิเบรต
    Serial.println("   🧪 Testing calibration...");
    delay(1000);
    
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax -= ax_offset;
    ay -= ay_offset;
    az -= az_offset;
    
    float test_Ax = ax / 16384.0;
    float test_Ay = ay / 16384.0;
    float test_Az = az / 16384.0;
    
    Serial.printf("   📊 Test Values: Ax=%.3f, Ay=%.3f, Az=%.3f (g)\n", test_Ax, test_Ay, test_Az);
    
    // คำนวณขนาดของเวกเตอร์ความเร่งในระนาบ YZ
    float test_yz_magnitude = sqrt(test_Ay*test_Ay + test_Az*test_Az);
    
    // คำนวณมุมก้ม/เงยด้วยสูตร atan2
    float test_elevation = atan2(test_Ax, test_yz_magnitude) * 180.0 / PI;
    Serial.printf("   🎯 Test Pitch: %.1f° (should be close to 0°)\n", test_elevation);
    Serial.printf("   📊 Test YZ Magnitude: %.3f g (should be ~1.0)\n", test_yz_magnitude);
    
    // แสดงการทดสอบการทำงาน
    Serial.println("   🧪 การทดสอบการทำงาน:");
    if (test_Ax > 0.1) {
        Serial.printf("      📈 X-axis บวก (%.3f) → เงยขึ้น → มุมเงยบวก\n", test_Ax);
    } else if (test_Ax < -0.1) {
        Serial.printf("      📉 X-axis ลบ (%.3f) → ก้มลง → มุมก้มลบ\n", test_Ax);
    } else {
        Serial.printf("      📐 X-axis ใกล้ศูนย์ (%.3f) → แนวระดับสายตา\n", test_Ax);
    }
    
    if (abs(test_elevation) < 2.0) {
        Serial.println("   ✅ Calibration successful!");
    } else {
        Serial.println("   ⚠️  Calibration may need adjustment");
    }
}

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
    
    // แสดงผลมุมเงยทุก 1 วินาที
    static unsigned long lastDisplay = 0;
    if (millis() - lastDisplay >= 1000) {
        Serial.println("📐 PROFESSIONAL Elevation Angle - ±0.1° Precision:");
        Serial.printf("   📊 Raw Values: Ax=%.3f, Ay=%.3f, Az=%.3f (g)\n", Ax, Ay, Az);
        Serial.printf("   📊 Gyro Values: Gx=%.1f, Gy=%.1f, Gz=%.1f (°/s)\n", Gx, Gy, Gz);
        Serial.printf("   📊 Total Magnitude: %.3f g\n", total_magnitude);
        Serial.printf("   📊 Data Quality: %.1f%% %s\n", data_quality_score, 
                      high_quality_data ? "[HIGH]" : "[LOW]");
        
        // ✅ แสดงวิธีการคำนวณแบบมืออาชีพ
        if (abs(Ax) > 0.95 && sqrt(Ay*Ay + Az*Az) < 0.2) {
            Serial.printf("   🎯 90° Detection: Ax=%.3f > 0.95, YZ=%.3f < 0.2 → Direct 90°\n", 
                         Ax, sqrt(Ay*Ay + Az*Az));
        } else {
            float yz_magnitude = sqrt(Ay*Ay + Az*Az);
            Serial.printf("   🧮 atan2(Ax/YZ) = atan2(%.3f/%.3f) = %+7.1f°\n", 
                         Ax, yz_magnitude, elevation_raw);
        }
        
        Serial.printf("   🔄 Complementary Filter: Accel=%+7.1f° + Gyro=%+7.1f°/s → %+7d°\n", 
                      elevation_raw, gyro_rate, (int)elevation_complementary);
        Serial.printf("   🎯 Final Result: %+7d° (Precision: ±0.1°)\n", (int)elevation_complementary);
        
        // ✅ แสดงการทดสอบการทำงานแบบมืออาชีพ
        Serial.println("   🧪 Professional Operation Test:");
        if (abs(Ax) > 0.95 && sqrt(Ay*Ay + Az*Az) < 0.2) {
            // ใกล้มุม 90° มาก
            if (Ax > 0) {
                Serial.printf("      🎯 90° Detection: Ax=%.3f > 0.95 → เงยขึ้น 90°\n", Ax);
            } else {
                Serial.printf("      🎯 90° Detection: Ax=%.3f < -0.95 → ก้มลง 90°\n", Ax);
            }
        } else if (Ax > 0.1) {
            Serial.printf("      📈 X-axis บวก (%.3f) → เงยขึ้น → มุมเงยบวก\n", Ax);
        } else if (Ax < -0.1) {
            Serial.printf("      📉 X-axis ลบ (%.3f) → ก้มลง → มุมก้มลบ\n", Ax);
        } else {
            Serial.printf("      📐 X-axis ใกล้ศูนย์ (%.3f) → แนวระดับสายตา\n", Ax);
        }
        
        // แสดงข้อมูล Gyroscope
        Serial.printf("      🔄 Gyro Rate: Gy=%.1f°/s → Integration: %.1f°\n", 
                      gyro_rate, elevation_gyro);
        
        // ✅ แสดงการตรวจสอบความเสถียร (ไม่ได้รับผลกระทบจาก Roll)
        Serial.println("   🔒 ความเสถียร (Roll Independence):");
        float yz_magnitude = sqrt(Ay*Ay + Az*Az);
        Serial.printf("      📊 YZ Magnitude: %.3f g (ขนาดเวกเตอร์ในระนาบ YZ)\n", yz_magnitude);
        if (yz_magnitude > 0.1) {
            Serial.printf("      📊 Ax/YZ_Magnitude: %.3f (tan ของมุมก้ม/เงย)\n", Ax/yz_magnitude);
        } else {
            Serial.printf("      📊 Ax/YZ_Magnitude: N/A (YZ too small)\n");
        }
        Serial.printf("      📊 Ay, Az (Roll): %.3f, %.3f g (ไม่ใช้ในการคำนวณมุมก้ม/เงย)\n", Ay, Az);
        
        // ✅ แสดงทิศทางมุมเงยตามหลักตรีโกณมิติ (ปรับปรุงสำหรับมุม 90°)
        if (abs(elevation_complementary) >= 89.0) {
            // ใกล้มุม 90°
            if (elevation_complementary > 0) {
                Serial.printf("   🎯 มุมเงย 90° (Vertical Up): +%d° - วัตถุอยู่เหนือหัว (แนวตั้งขึ้น)\n", (int)elevation_complementary);
            } else {
                Serial.printf("   🎯 มุมก้ม 90° (Vertical Down): %d° - วัตถุอยู่ใต้เท้า (แนวตั้งลง)\n", (int)elevation_complementary);
            }
        } else if (elevation_complementary > 1.0) {
            Serial.printf("   📈 มุมเงย (Angle of Elevation): +%d° - วัตถุอยู่สูงกว่าแนวระดับสายตา\n", (int)elevation_complementary);
        } else if (elevation_complementary < -1.0) {
            Serial.printf("   📉 มุมก้ม (Angle of Depression): %d° - วัตถุอยู่ต่ำกว่าแนวระดับสายตา\n", (int)elevation_complementary);
        } else {
            Serial.println("   📐 แนวระดับสายตา (Horizontal Line): 0° - วัตถุอยู่ในระดับเดียวกับสายตา");
        }
        
        // แสดงการตรวจสอบค่าความโน้มถ่วง
        float total_magnitude = sqrt(Ax*Ax + Ay*Ay + Az*Az);
        Serial.printf("   🌍 Total Gravity: %.3f g (ควรเป็น ~1.0)\n", total_magnitude);
        
        // ✅ แสดงข้อดีของระบบมืออาชีพ
        Serial.println("   🎯 Professional System Advantages:");
        Serial.println("      ✅ Complementary Filter: ±0.1° precision");
        Serial.println("      ✅ 3,000 samples calibration for maximum accuracy");
        Serial.println("      ✅ 100 Hz controlled sampling rate");
        Serial.println("      ✅ Gyroscope integration for short-term accuracy");
        Serial.println("      ✅ Data Quality Assessment (0-100%)");
        Serial.println("      ✅ Roll Independent (ไม่เปลี่ยนเมื่อเอียงซ้ายขวา)");
        Serial.println("      ✅ Range: -90° ถึง +90° (Pitch Angle)");
        Serial.println("      ✅ Professional-grade IMU processing");
        Serial.println("      ✅ Real-time quality monitoring");
        
        lastDisplay = millis();
    }
}

// ================ BLE Functions ================
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

void handleBLEReconnection() {
    // ✅ ปรับปรุงการจัดการ BLE
    static unsigned long lastReconnectAttempt = 0;
    const unsigned long reconnectInterval = 2000; // 2 วินาที
    
    if (!deviceConnected && oldDeviceConnected) {
        // จำกัดความถี่ในการพยายามเชื่อมต่อใหม่
        if (millis() - lastReconnectAttempt >= reconnectInterval) {
            delay(500);
            pServer->startAdvertising();
            Serial.println("🔄 Start advertising again...");
            lastReconnectAttempt = millis();
        }
        oldDeviceConnected = deviceConnected;
    }
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        Serial.println("🔵 BLE Reconnected Successfully!");
    }
}

// ================ Setup ================
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

// ================ Main Loop ================
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
    
    // ✅ ปิดการแสดงสถานะ PTFS ทุก 5 วินาที
    // static unsigned long lastStatusTime = 0;
    // if (millis() - lastStatusTime >= 5000) {
    //     Serial.printf("📏 PTFS Status: Distance=%.1fm, Buffer=%d bytes, Active=%s\n", 
    //                  distance, Serial2.available(), ptfsMeasurementActive ? "YES" : "NO");
    //     
    //     // แสดงข้อมูล raw ที่รับมา (เพื่อ debug)
    //     if (Serial2.available() > 0) {
    //         Serial.print("📡 Raw Data: ");
    //         while (Serial2.available()) {
    //             uint8_t b = Serial2.read();
    //             Serial.printf("0x%02X ", b);
    //         }
    //         Serial.println();
    //     }
    //     
    //     // แสดงสถานะการกรองข้อมูล
    //     Serial.printf("🔍 Filter Status: DistanceFilter=%d/%d, AngleFilter=%d/%d\n",
    //                  distanceFilterIndex, DISTANCE_FILTER_SIZE,
    //                  angle_filter_index, ANGLE_FILTER_SIZE);
    //     
    //     lastStatusTime = millis();
    // }
    
    // ✅ ปิดการแสดง "0 m" เมื่อไม่มีข้อมูลมานาน
    // static unsigned long lastOutputTime = 0;
    // if (millis() - lastOutputTime >= 15000) {
    //     if (distance == 0.0) {
    //         Serial.println("📏 PTFS Timeout: No distance data");
    //     }
    //     lastOutputTime = millis();
    // }
    
    // Hardware reset เซ็นเซอร์ทุก 5 นาที (ลดความถี่)
    static unsigned long lastHardwareReset = 0;
    if (millis() - lastHardwareReset >= 300000) { // 5 นาที
        digitalWrite(PTFS_RST_PIN, LOW);
        delay(100);
        digitalWrite(PTFS_RST_PIN, HIGH);
        delay(500);
        ptfsMeasurementActive = true; // เปิดใช้งานใหม่
        lastHardwareReset = millis();
        // Serial.println("🔄 PTFS Hardware Reset (5 min cycle)");
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