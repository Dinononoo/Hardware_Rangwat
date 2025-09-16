# คำอธิบายแต่ละบรรทัดของ Source Code - แบบละเอียด

## ภาพรวมของระบบ
ระบบนี้เป็นระบบวัดมุมเงย/มุมก้ม (Elevation/Depression Angle) แบบมืออาชีพด้วยความแม่นยำ ±0.1° โดยใช้:
- **ESP32** เป็นไมโครคอนโทรลเลอร์หลัก
- **MPU6050** สำหรับวัดมุมเงยด้วย Accelerometer + Gyroscope
- **PTFS Distance Sensor** สำหรับวัดระยะทาง
- **Bluetooth Low Energy (BLE)** สำหรับส่งข้อมูลไปยังมือถือ

## ส่วนที่ 1: Header Files และ Pin Configuration

### บรรทัด 1-12: Header Files
```cpp
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
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 3**: `#include <Arduino.h>` - ไฟล์หลักของ Arduino framework สำหรับ ESP32
- **บรรทัด 4**: `#include <driver/gpio.h>` - ไลบรารีสำหรับควบคุม GPIO pins ของ ESP32
- **บรรทัด 5**: `#include "Wire.h"` - ไลบรารีสำหรับการสื่อสาร I2C
- **บรรทัด 6**: `#include "I2Cdev.h"` - ไลบรารีช่วยสำหรับการสื่อสาร I2C
- **บรรทัด 7**: `#include "MPU6050.h"` - ไลบรารีสำหรับเซ็นเซอร์ MPU6050
- **บรรทัด 8**: `#include "BLEDevice.h"` - ไลบรารีหลักสำหรับ BLE
- **บรรทัด 9**: `#include "BLEServer.h"` - ไลบรารีสำหรับสร้าง BLE Server
- **บรรทัด 10**: `#include "BLEUtils.h"` - ฟังก์ชันช่วยสำหรับ BLE
- **บรรทัด 11**: `#include "BLE2902.h"` - Descriptor สำหรับ BLE characteristics
- **บรรทัด 12**: `#include "esp_bt.h"` - ไลบรารี Bluetooth ของ ESP32

### บรรทัด 14-22: Pin Configuration
```cpp
#define SDA_PIN 21
#define SCL_PIN 22

#define PTFS_RX_PIN 16
#define PTFS_TX_PIN 17
#define PTFS_PWR_PIN 2
#define PTFS_RST_PIN 4
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 14**: `#define SDA_PIN 21` - กำหนดขา SDA สำหรับ I2C เป็น GPIO21
- **บรรทัด 15**: `#define SCL_PIN 22` - กำหนดขา SCL สำหรับ I2C เป็น GPIO22
- **บรรทัด 17**: `#define PTFS_RX_PIN 16` - กำหนดขา RX สำหรับรับข้อมูลจาก PTFS เป็น GPIO16
- **บรรทัด 18**: `#define PTFS_TX_PIN 17` - กำหนดขา TX สำหรับส่งคำสั่งไป PTFS เป็น GPIO17
- **บรรทัด 19**: `#define PTFS_PWR_PIN 2` - กำหนดขา Power Enable สำหรับ PTFS เป็น GPIO2
- **บรรทัด 20**: `#define PTFS_RST_PIN 4` - กำหนดขา Reset สำหรับ PTFS เป็น GPIO4

### บรรทัด 24-27: BLE Configuration
```cpp
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME         "ESP32_LANDSLIDE_MOCK"
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 24**: `#define SERVICE_UUID` - กำหนด UUID ของ BLE Service
- **บรรทัด 25**: `#define CHARACTERISTIC_UUID` - กำหนด UUID ของ BLE Characteristic
- **บรรทัด 26**: `#define DEVICE_NAME` - กำหนดชื่ออุปกรณ์ BLE

## ส่วนที่ 2: Global Variables

### บรรทัด 29-31: MPU6050 Objects
```cpp
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 29**: `MPU6050 mpu;` - สร้างออบเจ็กต์ MPU6050 สำหรับควบคุมเซ็นเซอร์
- **บรรทัด 30**: `int16_t ax, ay, az, gx, gy, gz;` - ตัวแปรเก็บข้อมูลดิบจาก MPU6050 (accelerometer และ gyroscope)

### บรรทัด 33-36: Advanced Precision System
```cpp
float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;
float gx_filtered = 0, gy_filtered = 0, gz_filtered = 0;
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 33**: `float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;` - ตัวแปรเก็บข้อมูล accelerometer ที่ผ่านการกรองแล้ว
- **บรรทัด 34**: `float gx_filtered = 0, gy_filtered = 0, gz_filtered = 0;` - ตัวแปรเก็บข้อมูล gyroscope ที่ผ่านการกรองแล้ว

### บรรทัด 38-41: Calibration Variables
```cpp
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
bool calibrated = false;
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 38**: `int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;` - ตัวแปรเก็บค่า offset สำหรับ accelerometer
- **บรรทัด 39**: `int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;` - ตัวแปรเก็บค่า offset สำหรับ gyroscope
- **บรรทัด 40**: `bool calibrated = false;` - ตัวแปรบอกว่าทำการคาลิเบรตแล้วหรือยัง

### บรรทัด 43-47: Professional Elevation System
```cpp
float elevation_raw = 0.0;
float elevation_gyro = 0.0;
float elevation_complementary = 0.0;
float elevation_previous = 0.0;
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 43**: `float elevation_raw = 0.0;` - มุมเงยดิบจาก accelerometer
- **บรรทัด 44**: `float elevation_gyro = 0.0;` - มุมเงยจากการรวมข้อมูล gyroscope
- **บรรทัด 45**: `float elevation_complementary = 0.0;` - มุมเงยสุดท้ายจาก complementary filter
- **บรรทัด 46**: `float elevation_previous = 0.0;` - มุมเงยครั้งก่อนหน้า

### บรรทัด 49-53: Complementary Filter Parameters
```cpp
const float ALPHA = 0.98;
const float BETA = 0.02;
const float GYRO_SENSITIVITY = 131.0;
const float DT = 0.01;
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 49**: `const float ALPHA = 0.98;` - น้ำหนักของ gyroscope ใน complementary filter
- **บรรทัด 50**: `const float BETA = 0.02;` - น้ำหนักของ accelerometer ใน complementary filter
- **บรรทัด 51**: `const float GYRO_SENSITIVITY = 131.0;` - ความไวของ gyroscope (LSB/°/s)
- **บรรทัด 52**: `const float DT = 0.01;` - ช่วงเวลาสำหรับการรวมข้อมูล (10ms = 100Hz)

### บรรทัด 55-59: Data Quality Assessment
```cpp
float data_quality_score = 0.0;
float acceleration_magnitude = 0.0;
float gyro_magnitude = 0.0;
bool high_quality_data = false;
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 55**: `float data_quality_score = 0.0;` - คะแนนคุณภาพข้อมูล (0-100%)
- **บรรทัด 56**: `float acceleration_magnitude = 0.0;` - ขนาดของเวกเตอร์ความเร่ง
- **บรรทัด 57**: `float gyro_magnitude = 0.0;` - ขนาดของเวกเตอร์ gyroscope
- **บรรทัด 58**: `bool high_quality_data = false;` - ตัวแปรบอกว่าข้อมูลมีคุณภาพสูงหรือไม่

### บรรทัด 61-63: Sampling Rate Control
```cpp
unsigned long last_sample_time = 0;
const unsigned long SAMPLE_INTERVAL = 10;
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 61**: `unsigned long last_sample_time = 0;` - เวลาที่อ่านข้อมูลครั้งล่าสุด
- **บรรทัด 62**: `const unsigned long SAMPLE_INTERVAL = 10;` - ช่วงเวลาสำหรับการอ่านข้อมูล (10ms = 100Hz)

## ส่วนที่ 3: Functions

### ฟังก์ชัน calculateDataQuality (บรรทัด 66-97)
```cpp
float calculateDataQuality(float ax, float ay, float az, float gx, float gy, float gz) {
    acceleration_magnitude = sqrt(ax*ax + ay*ay + az*az);
    gyro_magnitude = sqrt(gx*gx + gy*gy + gz*gz);
    
    float quality = 100.0;
    
    float accel_error = abs(acceleration_magnitude - 1.0);
    if (accel_error > 0.1) {
        quality -= accel_error * 200;
    }
    
    if (gyro_magnitude > 50.0) {
        quality -= (gyro_magnitude - 50.0) * 0.5;
    }
    
    if (acceleration_magnitude < 0.5 || acceleration_magnitude > 2.0) {
        quality = 0.0;
    }
    
    if (quality < 0.0) quality = 0.0;
    if (quality > 100.0) quality = 100.0;
    
    return quality;
}
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 66**: `float calculateDataQuality(...)` - ฟังก์ชันคำนวณคุณภาพข้อมูล
- **บรรทัด 67**: `acceleration_magnitude = sqrt(ax*ax + ay*ay + az*az);` - คำนวณขนาดของเวกเตอร์ความเร่ง
- **บรรทัด 68**: `gyro_magnitude = sqrt(gx*gx + gy*gy + gz*gz);` - คำนวณขนาดของเวกเตอร์ gyroscope
- **บรรทัด 70**: `float quality = 100.0;` - เริ่มต้นคะแนนคุณภาพที่ 100%
- **บรรทัด 72**: `float accel_error = abs(acceleration_magnitude - 1.0);` - คำนวณความผิดพลาดของความเร่ง
- **บรรทัด 73-75**: ตรวจสอบและลดคะแนนถ้าความเร่งผิดปกติ
- **บรรทัด 77-79**: ตรวจสอบและลดคะแนนถ้า gyroscope หมุนเร็วเกินไป
- **บรรทัด 81-83**: ตั้งคะแนนเป็น 0 ถ้าข้อมูลไม่สมเหตุสมผล
- **บรรทัด 85-86**: จำกัดคะแนนให้อยู่ในช่วง 0-100%
- **บรรทัด 88**: `return quality;` - ส่งคืนคะแนนคุณภาพ

### ฟังก์ชัน applyComplementaryFilter (บรรทัด 100-111)
```cpp
float applyComplementaryFilter(float accel_angle, float gyro_rate, float dt) {
    elevation_gyro = elevation_previous + gyro_rate * dt;
    elevation_complementary = ALPHA * elevation_gyro + BETA * accel_angle;
    elevation_previous = elevation_complementary;
    return elevation_complementary;
}
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 100**: `float applyComplementaryFilter(...)` - ฟังก์ชันใช้ complementary filter
- **บรรทัด 101**: `elevation_gyro = elevation_previous + gyro_rate * dt;` - รวมข้อมูล gyroscope
- **บรรทัด 102**: `elevation_complementary = ALPHA * elevation_gyro + BETA * accel_angle;` - รวมข้อมูลทั้งสองแบบ
- **บรรทัด 103**: `elevation_previous = elevation_complementary;` - เก็บค่าสำหรับครั้งต่อไป
- **บรรทัด 104**: `return elevation_complementary;` - ส่งคืนมุมเงยสุดท้าย

## ส่วนที่ 4: BLE Variables และ Callbacks

### บรรทัด 113-117: BLE Variables
```cpp
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 113**: `BLEServer* pServer = NULL;` - ตัวแปรสำหรับ BLE Server
- **บรรทัด 114**: `BLECharacteristic* pCharacteristic = NULL;` - ตัวแปรสำหรับ BLE Characteristic
- **บรรทัด 115**: `bool deviceConnected = false;` - ตัวแปรบอกว่ามีการเชื่อมต่อหรือไม่
- **บรรทัด 116**: `bool oldDeviceConnected = false;` - ตัวแปรเก็บสถานะการเชื่อมต่อครั้งก่อน

### บรรทัด 119-123: Data Variables
```cpp
float distance = 0.0;
int currentMode = 1;
unsigned long lastSendTime = 0;
const long sendInterval = 1000;
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 119**: `float distance = 0.0;` - ระยะทางจาก PTFS sensor
- **บรรทัด 120**: `int currentMode = 1;` - โหมดปัจจุบัน
- **บรรทัด 121**: `unsigned long lastSendTime = 0;` - เวลาส่งข้อมูลครั้งล่าสุด
- **บรรทัด 122**: `const long sendInterval = 1000;` - ช่วงเวลาส่งข้อมูล (1000ms)

### บรรทัด 125-134: Error Handling Variables
```cpp
bool mpu6050Healthy = false;
bool ptfsHealthy = false;
bool bleHealthy = false;
unsigned long lastMPU6050Check = 0;
unsigned long lastPTFSCheck = 0;
unsigned long lastBLECheck = 0;
const unsigned long healthCheckInterval = 5000;
int errorCount = 0;
const int maxErrorCount = 10;
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 125**: `bool mpu6050Healthy = false;` - สถานะสุขภาพของ MPU6050
- **บรรทัด 126**: `bool ptfsHealthy = false;` - สถานะสุขภาพของ PTFS
- **บรรทัด 127**: `bool bleHealthy = false;` - สถานะสุขภาพของ BLE
- **บรรทัด 128**: `unsigned long lastMPU6050Check = 0;` - เวลาตรวจสอบ MPU6050 ครั้งล่าสุด
- **บรรทัด 129**: `unsigned long lastPTFSCheck = 0;` - เวลาตรวจสอบ PTFS ครั้งล่าสุด
- **บรรทัด 130**: `unsigned long lastBLECheck = 0;` - เวลาตรวจสอบ BLE ครั้งล่าสุด
- **บรรทัด 131**: `const unsigned long healthCheckInterval = 5000;` - ช่วงเวลาตรวจสอบสุขภาพ (5 วินาที)
- **บรรทัด 132**: `int errorCount = 0;` - จำนวนข้อผิดพลาด
- **บรรทัด 133**: `const int maxErrorCount = 10;` - จำนวนข้อผิดพลาดสูงสุด

### บรรทัด 136-151: PTFS Variables
```cpp
uint8_t ptfsBuffer[200];
int ptfsBufferIndex = 0;
bool ptfsMeasurementActive = false;
bool loopbackDetected = false;
unsigned long lastPTFSCommand = 0;
unsigned long lastPTFSAnalysis = 0;
unsigned long lastPTFSDisplayTime = 0;
const unsigned long ptfsCommandInterval = 2000;
const unsigned long ptfsAnalysisInterval = 1000;

#define DISTANCE_FILTER_SIZE 30
float distanceHistory[DISTANCE_FILTER_SIZE] = {0};
int distanceFilterIndex = 0;
bool distanceFilterFull = false;
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 136**: `uint8_t ptfsBuffer[200];` - บัฟเฟอร์สำหรับเก็บข้อมูลจาก PTFS
- **บรรทัด 137**: `int ptfsBufferIndex = 0;` - ดัชนีปัจจุบันในบัฟเฟอร์
- **บรรทัด 138**: `bool ptfsMeasurementActive = false;` - สถานะการวัดระยะทาง
- **บรรทัด 139**: `bool loopbackDetected = false;` - สถานะการตรวจจับ loopback
- **บรรทัด 140**: `unsigned long lastPTFSCommand = 0;` - เวลาส่งคำสั่ง PTFS ครั้งล่าสุด
- **บรรทัด 141**: `unsigned long lastPTFSAnalysis = 0;` - เวลาวิเคราะห์ข้อมูล PTFS ครั้งล่าสุด
- **บรรทัด 142**: `unsigned long lastPTFSDisplayTime = 0;` - เวลแสดงผล PTFS ครั้งล่าสุด
- **บรรทัด 143**: `const unsigned long ptfsCommandInterval = 2000;` - ช่วงเวลาส่งคำสั่ง PTFS (2 วินาที)
- **บรรทัด 144**: `const unsigned long ptfsAnalysisInterval = 1000;` - ช่วงเวลาวิเคราะห์ข้อมูล PTFS (1 วินาที)
- **บรรทัด 146**: `#define DISTANCE_FILTER_SIZE 30` - ขนาดของ filter สำหรับระยะทาง
- **บรรทัด 147**: `float distanceHistory[DISTANCE_FILTER_SIZE] = {0};` - array เก็บประวัติระยะทาง
- **บรรทัด 148**: `int distanceFilterIndex = 0;` - ดัชนีปัจจุบันใน filter
- **บรรทัด 149**: `bool distanceFilterFull = false;` - สถานะว่า filter เต็มหรือไม่

## ส่วนที่ 5: BLE Callbacks

### Class MyServerCallbacks (บรรทัด 154-164)
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

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 154**: `class MyServerCallbacks: public BLEServerCallbacks {` - สร้าง class สำหรับจัดการ BLE Server callbacks
- **บรรทัด 155**: `void onConnect(BLEServer* pServer) {` - ฟังก์ชันที่เรียกเมื่อมีการเชื่อมต่อ
- **บรรทัด 156**: `deviceConnected = true;` - ตั้งค่าสถานะการเชื่อมต่อเป็น true
- **บรรทัด 157**: `Serial.println("🔵 อุปกรณ์เชื่อมต่อ!");` - แสดงข้อความการเชื่อมต่อ
- **บรรทัด 160**: `void onDisconnect(BLEServer* pServer) {` - ฟังก์ชันที่เรียกเมื่อมีการตัดการเชื่อมต่อ
- **บรรทัด 161**: `deviceConnected = false;` - ตั้งค่าสถานะการเชื่อมต่อเป็น false
- **บรรทัด 162**: `Serial.println("🔴 อุปกรณ์ตัดการเชื่อมต่อ!");` - แสดงข้อความการตัดการเชื่อมต่อ

### Class MyCharacteristicCallbacks (บรรทัด 166-180)
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

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 166**: `class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {` - สร้าง class สำหรับจัดการ BLE Characteristic callbacks
- **บรรทัด 167**: `void onWrite(BLECharacteristic *pCharacteristic) {` - ฟังก์ชันที่เรียกเมื่อมีการเขียนข้อมูล
- **บรรทัด 168**: `uint8_t* data = pCharacteristic->getData();` - ดึงข้อมูลที่เขียนมา
- **บรรทัด 169**: `size_t len = pCharacteristic->getLength();` - ดึงความยาวของข้อมูล
- **บรรทัด 170**: `String value = "";` - สร้างตัวแปร string สำหรับเก็บข้อมูล
- **บรรทัด 171-173**: วนลูปแปลงข้อมูลเป็น string
- **บรรทัด 174-175**: แสดงข้อความที่รับมา
- **บรรทัด 177-178**: ตรวจสอบและเปลี่ยนโหมดตามคำสั่ง

## ส่วนที่ 6: System Health Monitoring

### ฟังก์ชัน checkMPU6050Health (บรรทัด 184-193)
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

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 184**: `bool checkMPU6050Health() {` - ฟังก์ชันตรวจสอบสุขภาพของ MPU6050
- **บรรทัด 185**: `if (mpu.testConnection()) {` - ตรวจสอบการเชื่อมต่อกับ MPU6050
- **บรรทัด 186**: `mpu6050Healthy = true;` - ตั้งค่าสถานะสุขภาพเป็น true
- **บรรทัด 187**: `return true;` - ส่งคืน true
- **บรรทัด 188**: `} else {` - กรณีที่การเชื่อมต่อล้มเหลว
- **บรรทัด 189**: `mpu6050Healthy = false;` - ตั้งค่าสถานะสุขภาพเป็น false
- **บรรทัด 190**: `Serial.println("❌ MPU6050 Health Check Failed!");` - แสดงข้อความข้อผิดพลาด
- **บรรทัด 191**: `return false;` - ส่งคืน false

### ฟังก์ชัน checkPTFSHealth (บรรทัด 195-205)
```cpp
bool checkPTFSHealth() {
    if (millis() - lastPTFSAnalysis < 10000) {
        ptfsHealthy = true;
        return true;
    } else {
        ptfsHealthy = false;
        Serial.println("⚠️ PTFS Health Check Failed - No Response!");
        return false;
    }
}
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 195**: `bool checkPTFSHealth() {` - ฟังก์ชันตรวจสอบสุขภาพของ PTFS
- **บรรทัด 196**: `if (millis() - lastPTFSAnalysis < 10000) {` - ตรวจสอบว่ามีการตอบสนองใน 10 วินาทีที่ผ่านมาหรือไม่
- **บรรทัด 197**: `ptfsHealthy = true;` - ตั้งค่าสถานะสุขภาพเป็น true
- **บรรทัด 198**: `return true;` - ส่งคืน true
- **บรรทัด 199**: `} else {` - กรณีที่ไม่มีข้อมูลใน 10 วินาที
- **บรรทัด 200**: `ptfsHealthy = false;` - ตั้งค่าสถานะสุขภาพเป็น false
- **บรรทัด 201**: `Serial.println("⚠️ PTFS Health Check Failed - No Response!");` - แสดงข้อความข้อผิดพลาด
- **บรรทัด 202**: `return false;` - ส่งคืน false

### ฟังก์ชัน checkBLEHealth (บรรทัด 207-216)
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

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 207**: `bool checkBLEHealth() {` - ฟังก์ชันตรวจสอบสุขภาพของ BLE
- **บรรทัด 208**: `if (pServer && pCharacteristic) {` - ตรวจสอบว่า pServer และ pCharacteristic มีค่าหรือไม่
- **บรรทัด 209**: `bleHealthy = true;` - ตั้งค่าสถานะสุขภาพเป็น true
- **บรรทัด 210**: `return true;` - ส่งคืน true
- **บรรทัด 211**: `} else {` - กรณีที่ pServer หรือ pCharacteristic เป็น NULL
- **บรรทัด 212**: `bleHealthy = false;` - ตั้งค่าสถานะสุขภาพเป็น false
- **บรรทัด 213**: `Serial.println("❌ BLE Health Check Failed!");` - แสดงข้อความข้อผิดพลาด
- **บรรทัด 214**: `return false;` - ส่งคืน false

### ฟังก์ชัน performHealthCheck (บรรทัด 218-248)
```cpp
void performHealthCheck() {
    static unsigned long lastHealthCheck = 0;
    
    if (millis() - lastHealthCheck >= healthCheckInterval) {
        bool mpuOK = checkMPU6050Health();
        bool ptfsOK = checkPTFSHealth();
        bool bleOK = checkBLEHealth();
        
        if (!mpuOK || !ptfsOK || !bleOK) {
            errorCount++;
            Serial.printf("⚠️ Health Check Failed! Error Count: %d/%d\n", errorCount, maxErrorCount);
            
            if (errorCount >= maxErrorCount) {
                Serial.println("🔄 Too Many Errors - Restarting System...");
                ESP.restart();
            }
        } else {
            errorCount = 0;
        }
        
        Serial.printf("💾 Free Heap: %d bytes\n", ESP.getFreeHeap());
        if (ESP.getFreeHeap() < 10000) {
            Serial.println("⚠️ Low Memory Warning!");
        }
        
        lastHealthCheck = millis();
    }
}
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 218**: `void performHealthCheck() {` - ฟังก์ชันตรวจสอบสุขภาพของระบบ
- **บรรทัด 219**: `static unsigned long lastHealthCheck = 0;` - ตัวแปรเก็บเวลาตรวจสอบครั้งล่าสุด
- **บรรทัด 221**: `if (millis() - lastHealthCheck >= healthCheckInterval) {` - ตรวจสอบว่าถึงเวลาตรวจสอบหรือไม่
- **บรรทัด 222**: `bool mpuOK = checkMPU6050Health();` - ตรวจสอบสุขภาพ MPU6050
- **บรรทัด 223**: `bool ptfsOK = checkPTFSHealth();` - ตรวจสอบสุขภาพ PTFS
- **บรรทัด 224**: `bool bleOK = checkBLEHealth();` - ตรวจสอบสุขภาพ BLE
- **บรรทัด 226**: `if (!mpuOK || !ptfsOK || !bleOK) {` - ตรวจสอบว่ามีข้อผิดพลาดหรือไม่
- **บรรทัด 227**: `errorCount++;` - เพิ่มจำนวนข้อผิดพลาด
- **บรรทัด 228**: `Serial.printf("⚠️ Health Check Failed! Error Count: %d/%d\n", errorCount, maxErrorCount);` - แสดงข้อความข้อผิดพลาด
- **บรรทัด 230-233**: ตรวจสอบและรีสตาร์ทระบบถ้ามีข้อผิดพลาดมากเกินไป
- **บรรทัด 234-236**: รีเซ็ตตัวนับข้อผิดพลาดถ้าระบบปกติ
- **บรรทัด 238-241**: แสดงสถานะหน่วยความจำ
- **บรรทัด 243**: `lastHealthCheck = millis();` - อัปเดตเวลาตรวจสอบครั้งล่าสุด


