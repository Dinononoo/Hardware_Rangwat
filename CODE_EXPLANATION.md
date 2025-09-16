# อธิบาย Source Code ESP32 + MPU6050 + PTFS Distance Sensor + BLE

## ภาพรวมของระบบ
ระบบนี้เป็นระบบวัดมุมเงย/ก้ม (Elevation/Depression Angle) และระยะทางแบบมืออาชีพ ที่ใช้ ESP32 ร่วมกับเซ็นเซอร์ MPU6050 และ PTFS Distance Sensor พร้อมการส่งข้อมูลผ่าน BLE

## ส่วนประกอบหลัก

### 1. Header Files และ Include (บรรทัด 1-12)
```cpp
// ESP32 + MPU6050 + PTFS Distance Sensor + BLE
// ระบบเซ็นเซอร์รวมสำหรับการตรวจสอบดินถล่ม
#include <Arduino.h>        // ไลบรารีหลักของ Arduino สำหรับ ESP32
#include <driver/gpio.h>    // ไลบรารีควบคุม GPIO ของ ESP32
#include "Wire.h"           // ไลบรารี I2C communication
#include "I2Cdev.h"         // ไลบรารีสำหรับ I2C devices
#include "MPU6050.h"        // ไลบรารีเซ็นเซอร์ MPU6050
#include "BLEDevice.h"      // ไลบรารี BLE หลัก
#include "BLEServer.h"      // ไลบรารี BLE Server
#include "BLEUtils.h"       // ไลบรารี BLE utilities
#include "BLE2902.h"        // ไลบรารี BLE descriptor
#include "esp_bt.h"         // ไลบรารี Bluetooth ของ ESP32
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 1**: คอมเมนต์อธิบายชื่อโปรเจคและส่วนประกอบหลัก
- **บรรทัด 2**: คอมเมนต์อธิบายวัตถุประสงค์ของระบบ (ตรวจสอบดินถล่ม)
- **บรรทัด 3**: `#include <Arduino.h>` - ไลบรารีหลักของ Arduino ที่มีฟังก์ชันพื้นฐาน เช่น `Serial`, `delay()`, `millis()`, `pinMode()`, `digitalWrite()`
- **บรรทัด 4**: `#include <driver/gpio.h>` - ไลบรารีเฉพาะของ ESP32 สำหรับควบคุม GPIO pins แบบละเอียด เช่น `gpio_set_drive_capability()`, `gpio_set_pull_mode()`
- **บรรทัด 5**: `#include "Wire.h"` - ไลบรารีสำหรับการสื่อสาร I2C ใช้เชื่อมต่อกับ MPU6050
- **บรรทัด 6**: `#include "I2Cdev.h"` - ไลบรารีช่วยเหลือสำหรับการทำงานกับ I2C devices
- **บรรทัด 7**: `#include "MPU6050.h"` - ไลบรารีเฉพาะสำหรับเซ็นเซอร์ MPU6050 (Accelerometer + Gyroscope)
- **บรรทัด 8**: `#include "BLEDevice.h"` - ไลบรารีหลักของ BLE สำหรับ ESP32
- **บรรทัด 9**: `#include "BLEServer.h"` - ไลบรารีสำหรับสร้าง BLE Server
- **บรรทัด 10**: `#include "BLEUtils.h"` - ไลบรารีเครื่องมือช่วยเหลือสำหรับ BLE
- **บรรทัด 11**: `#include "BLE2902.h"` - ไลบรารีสำหรับ BLE Descriptor (ใช้สำหรับ Notify/Indicate)
- **บรรทัด 12**: `#include "esp_bt.h"` - ไลบรารี Bluetooth ของ ESP32 สำหรับการจัดการหน่วยความจำ

### 2. การกำหนด Pin Configuration (บรรทัด 14-22)
```cpp
// ================ การกำหนด Pin ================
#define SDA_PIN 21  // GPIO21 - เส้นข้อมูล I2C
#define SCL_PIN 22  // GPIO22 - เส้นนาฬิกา I2C

// Pin สำหรับเซ็นเซอร์ระยะทาง PTFS (ระบบ 3.3V)
#define PTFS_RX_PIN 16  // GPIO16 (RX2) - รับข้อมูลจาก PTFS TX
#define PTFS_TX_PIN 17  // GPIO17 (TX2) - ส่งคำสั่งไป PTFS RX
#define PTFS_PWR_PIN 2  // GPIO2 - เปิด/ปิดไฟ (จ่าย HIGH เพื่อเปิด PTFS)
#define PTFS_RST_PIN 4  // GPIO4 - Reset
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 14**: คอมเมนต์หัวข้อสำหรับการกำหนด Pin
- **บรรทัด 15**: `#define SDA_PIN 21` - กำหนดให้ GPIO21 เป็น SDA (Serial Data) สำหรับ I2C communication กับ MPU6050
- **บรรทัด 16**: `#define SCL_PIN 22` - กำหนดให้ GPIO22 เป็น SCL (Serial Clock) สำหรับ I2C communication กับ MPU6050
- **บรรทัด 17**: คอมเมนต์ว่างเพื่อแยกส่วน
- **บรรทัด 18**: คอมเมนต์อธิบาย Pin สำหรับ PTFS Distance Sensor
- **บรรทัด 19**: `#define PTFS_RX_PIN 16` - กำหนดให้ GPIO16 เป็น RX2 (Receive) รับข้อมูลจาก PTFS sensor
- **บรรทัด 20**: `#define PTFS_TX_PIN 17` - กำหนดให้ GPIO17 เป็น TX2 (Transmit) ส่งคำสั่งไปยัง PTFS sensor
- **บรรทัด 21**: `#define PTFS_PWR_PIN 2` - กำหนดให้ GPIO2 เป็น Power Enable pin สำหรับเปิด/ปิด PTFS sensor
- **บรรทัด 22**: `#define PTFS_RST_PIN 4` - กำหนดให้ GPIO4 เป็น Reset pin สำหรับรีเซ็ต PTFS sensor

### 3. BLE Configuration (บรรทัด 24-27)
```cpp
// ================ BLE Configuration ================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DEVICE_NAME         "ESP32_LANDSLIDE_MOCK"
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 24**: คอมเมนต์หัวข้อสำหรับการกำหนดค่า BLE
- **บรรทัด 25**: `#define SERVICE_UUID` - กำหนด UUID (Universally Unique Identifier) สำหรับ BLE Service ที่จะให้อุปกรณ์อื่นค้นหาและเชื่อมต่อได้
- **บรรทัด 26**: `#define CHARACTERISTIC_UUID` - กำหนด UUID สำหรับ BLE Characteristic ที่จะใช้ส่ง/รับข้อมูลระหว่าง ESP32 กับอุปกรณ์อื่น
- **บรรทัด 27**: `#define DEVICE_NAME` - กำหนดชื่อที่แสดงเมื่อค้นหา BLE device (จะปรากฏในแอปมือถือ)

### 4. Global Variables (บรรทัด 29-63)
```cpp
// ================ Global Variables ================
MPU6050 mpu;                    // วัตถุเซ็นเซอร์ MPU6050
int16_t ax, ay, az, gx, gy, gz; // ตัวแปรเก็บข้อมูลจาก MPU6050

// ระบบกรองข้อมูลแบบมืออาชีพ
float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;
float gx_filtered = 0, gy_filtered = 0, gz_filtered = 0;

// ตัวแปรคาลิเบรต - เพิ่มเป็น 3,000 samples
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
bool calibrated = false;

// ระบบวัดมุมเงยแบบมืออาชีพ - ความแม่นยำ ±0.1°
float elevation_raw = 0.0;           // มุมเงยดิบจาก accelerometer
float elevation_gyro = 0.0;          // มุมเงยจาก gyroscope integration
float elevation_complementary = 0.0; // ผลลัพธ์สุดท้ายจาก complementary filter
float elevation_previous = 0.0;      // มุมเงยก่อนหน้า

// พารามิเตอร์ Complementary Filter (ระดับมืออาชีพ)
const float ALPHA = 0.98;            // น้ำหนัก gyroscope (ความถี่สูง)
const float BETA = 0.02;             // น้ำหนัก accelerometer (ความถี่ต่ำ)
const float GYRO_SENSITIVITY = 131.0; // LSB/°/s สำหรับช่วง ±250°/s
const float DT = 0.01;               // อัตราการสุ่มตัวอย่าง 100 Hz (10ms)

// การประเมินคุณภาพข้อมูล
float data_quality_score = 0.0;      // คะแนนคุณภาพ 0-100%
float acceleration_magnitude = 0.0;  // ขนาดความเร่งรวม
float gyro_magnitude = 0.0;          // ขนาด gyro รวม
bool high_quality_data = false;      // ฟลากเกณฑ์คุณภาพ

// การควบคุมอัตราการสุ่มตัวอย่าง
unsigned long last_sample_time = 0;
const unsigned long SAMPLE_INTERVAL = 10; // 100 Hz = 10ms interval
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 29**: คอมเมนต์หัวข้อสำหรับตัวแปร Global
- **บรรทัด 30**: `MPU6050 mpu;` - สร้างวัตถุ (object) ของคลาส MPU6050 เพื่อควบคุมเซ็นเซอร์
- **บรรทัด 31**: `int16_t ax, ay, az, gx, gy, gz;` - ประกาศตัวแปรเก็บข้อมูลดิบจาก MPU6050 (ax,ay,az = accelerometer, gx,gy,gz = gyroscope)
- **บรรทัด 32**: คอมเมนต์ว่างเพื่อแยกส่วน
- **บรรทัด 33**: คอมเมนต์อธิบายระบบกรองข้อมูล
- **บรรทัด 34**: `float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;` - ตัวแปรเก็บข้อมูล accelerometer ที่ผ่านการกรองแล้ว
- **บรรทัด 35**: `float gx_filtered = 0, gy_filtered = 0, gz_filtered = 0;` - ตัวแปรเก็บข้อมูล gyroscope ที่ผ่านการกรองแล้ว
- **บรรทัด 36**: คอมเมนต์ว่างเพื่อแยกส่วน
- **บรรทัด 37**: คอมเมนต์อธิบายตัวแปรคาลิเบรต
- **บรรทัด 38**: `int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;` - ตัวแปรเก็บค่า offset ของ accelerometer สำหรับการคาลิเบรต
- **บรรทัด 39**: `int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;` - ตัวแปรเก็บค่า offset ของ gyroscope สำหรับการคาลิเบรต
- **บรรทัด 40**: `bool calibrated = false;` - ตัวแปรบอกว่าการคาลิเบรตเสร็จสิ้นแล้วหรือไม่
- **บรรทัด 41**: คอมเมนต์ว่างเพื่อแยกส่วน
- **บรรทัด 42**: คอมเมนต์อธิบายระบบวัดมุมเงย
- **บรรทัด 43**: `float elevation_raw = 0.0;` - ตัวแปรเก็บมุมเงยดิบที่คำนวณจาก accelerometer
- **บรรทัด 44**: `float elevation_gyro = 0.0;` - ตัวแปรเก็บมุมเงยที่คำนวณจากการรวมข้อมูล gyroscope
- **บรรทัด 45**: `float elevation_complementary = 0.0;` - ตัวแปรเก็บผลลัพธ์สุดท้ายจาก complementary filter
- **บรรทัด 46**: `float elevation_previous = 0.0;` - ตัวแปรเก็บมุมเงยก่อนหน้าเพื่อใช้ในการคำนวณครั้งถัดไป
- **บรรทัด 47**: คอมเมนต์ว่างเพื่อแยกส่วน
- **บรรทัด 48**: คอมเมนต์อธิบายพารามิเตอร์ Complementary Filter
- **บรรทัด 49**: `const float ALPHA = 0.98;` - ค่าคงที่สำหรับน้ำหนักของ gyroscope (98%) ในการรวมข้อมูล
- **บรรทัด 50**: `const float BETA = 0.02;` - ค่าคงที่สำหรับน้ำหนักของ accelerometer (2%) ในการรวมข้อมูล
- **บรรทัด 51**: `const float GYRO_SENSITIVITY = 131.0;` - ค่าคงที่สำหรับแปลงข้อมูล gyroscope เป็นองศาต่อวินาที
- **บรรทัด 52**: `const float DT = 0.01;` - ค่าคงที่สำหรับเวลาระหว่างการสุ่มตัวอย่าง (10ms = 100Hz)
- **บรรทัด 53**: คอมเมนต์ว่างเพื่อแยกส่วน
- **บรรทัด 54**: คอมเมนต์อธิบายการประเมินคุณภาพข้อมูล
- **บรรทัด 55**: `float data_quality_score = 0.0;` - ตัวแปรเก็บคะแนนคุณภาพข้อมูล (0-100%)
- **บรรทัด 56**: `float acceleration_magnitude = 0.0;` - ตัวแปรเก็บขนาดของเวกเตอร์ความเร่งรวม
- **บรรทัด 57**: `float gyro_magnitude = 0.0;` - ตัวแปรเก็บขนาดของเวกเตอร์ gyroscope รวม
- **บรรทัด 58**: `bool high_quality_data = false;` - ตัวแปรบอกว่าข้อมูลมีคุณภาพสูงหรือไม่
- **บรรทัด 59**: คอมเมนต์ว่างเพื่อแยกส่วน
- **บรรทัด 60**: คอมเมนต์อธิบายการควบคุมอัตราการสุ่มตัวอย่าง
- **บรรทัด 61**: `unsigned long last_sample_time = 0;` - ตัวแปรเก็บเวลาที่สุ่มตัวอย่างครั้งล่าสุด
- **บรรทัด 62**: `const unsigned long SAMPLE_INTERVAL = 10;` - ค่าคงที่สำหรับช่วงเวลาระหว่างการสุ่มตัวอย่าง (10ms)

### 5. ฟังก์ชันประเมินคุณภาพข้อมูล (บรรทัด 66-97)
```cpp
float calculateDataQuality(float ax, float ay, float az, float gx, float gy, float gz) {
    // คำนวณขนาดความเร่ง (ควรเป็น ~1g เมื่อหยุดนิ่ง)
    acceleration_magnitude = sqrt(ax*ax + ay*ay + az*az);
    
    // คำนวณขนาด gyroscope (ควรต่ำเมื่อหยุดนิ่ง)
    gyro_magnitude = sqrt(gx*gx + gy*gy + gz*gz);
    
    // การประเมินคุณภาพตามหลายปัจจัย
    float quality = 100.0;
    
    // ตรวจสอบขนาดความเร่ง (ควรใกล้ 1g)
    float accel_error = abs(acceleration_magnitude - 1.0);
    if (accel_error > 0.1) {
        quality -= accel_error * 200; // ลดคะแนนเมื่อความเร่งผิดปกติ
    }
    
    // ตรวจสอบขนาด gyroscope (ควรต่ำเมื่อหยุดนิ่ง)
    if (gyro_magnitude > 50.0) { // การหมุนสูง
        quality -= (gyro_magnitude - 50.0) * 0.5; // ลดคะแนนเมื่อหมุนมาก
    }
    
    // ตรวจสอบค่าที่สมเหตุสมผล
    if (acceleration_magnitude < 0.5 || acceleration_magnitude > 2.0) {
        quality = 0.0; // ข้อมูลไม่ถูกต้อง
    }
    
    // ให้แน่ใจว่าคุณภาพอยู่ระหว่าง 0-100%
    if (quality < 0.0) quality = 0.0;
    if (quality > 100.0) quality = 100.0;
    
    return quality;
}
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 66**: `float calculateDataQuality(...)` - ประกาศฟังก์ชันที่รับพารามิเตอร์ 6 ตัว (ax,ay,az,gx,gy,gz) และคืนค่าเป็น float
- **บรรทัด 67**: คอมเมนต์อธิบายการคำนวณขนาดความเร่ง
- **บรรทัด 68**: `acceleration_magnitude = sqrt(ax*ax + ay*ay + az*az);` - คำนวณขนาดของเวกเตอร์ความเร่งรวมโดยใช้สูตร √(x²+y²+z²)
- **บรรทัด 69**: คอมเมนต์ว่าง
- **บรรทัด 70**: คอมเมนต์อธิบายการคำนวณขนาด gyroscope
- **บรรทัด 71**: `gyro_magnitude = sqrt(gx*gx + gy*gy + gz*gz);` - คำนวณขนาดของเวกเตอร์ gyroscope รวม
- **บรรทัด 72**: คอมเมนต์ว่าง
- **บรรทัด 73**: คอมเมนต์อธิบายการประเมินคุณภาพ
- **บรรทัด 74**: `float quality = 100.0;` - เริ่มต้นคะแนนคุณภาพที่ 100% (เต็มคะแนน)
- **บรรทัด 75**: คอมเมนต์ว่าง
- **บรรทัด 76**: คอมเมนต์อธิบายการตรวจสอบขนาดความเร่ง
- **บรรทัด 77**: `float accel_error = abs(acceleration_magnitude - 1.0);` - คำนวณความแตกต่างระหว่างขนาดความเร่งกับ 1g (ค่าที่ควรเป็นเมื่อหยุดนิ่ง)
- **บรรทัด 78**: `if (accel_error > 0.1) {` - ตรวจสอบว่าความแตกต่างเกิน 0.1g หรือไม่
- **บรรทัด 79**: `quality -= accel_error * 200;` - ลดคะแนนคุณภาพตามความผิดพลาด (คูณ 200 เพื่อให้ผลกระทบชัดเจน)
- **บรรทัด 80**: `}` - ปิด if statement
- **บรรทัด 81**: คอมเมนต์ว่าง
- **บรรทัด 82**: คอมเมนต์อธิบายการตรวจสอบขนาด gyroscope
- **บรรทัด 83**: `if (gyro_magnitude > 50.0) {` - ตรวจสอบว่าการหมุนเกิน 50°/s หรือไม่ (ค่าสูงเมื่อมีการเคลื่อนไหว)
- **บรรทัด 84**: คอมเมนต์อธิบายการหมุนสูง
- **บรรทัด 85**: `quality -= (gyro_magnitude - 50.0) * 0.5;` - ลดคะแนนคุณภาพตามการหมุนที่เกิน 50°/s
- **บรรทัด 86**: `}` - ปิด if statement
- **บรรทัด 87**: คอมเมนต์ว่าง
- **บรรทัด 88**: คอมเมนต์อธิบายการตรวจสอบค่าที่สมเหตุสมผล
- **บรรทัด 89**: `if (acceleration_magnitude < 0.5 || acceleration_magnitude > 2.0) {` - ตรวจสอบว่าขนาดความเร่งอยู่นอกช่วง 0.5-2.0g หรือไม่
- **บรรทัด 90**: `quality = 0.0;` - ตั้งคะแนนคุณภาพเป็น 0% เมื่อข้อมูลไม่ถูกต้อง
- **บรรทัด 91**: คอมเมนต์อธิบายข้อมูลไม่ถูกต้อง
- **บรรทัด 92**: `}` - ปิด if statement
- **บรรทัด 93**: คอมเมนต์ว่าง
- **บรรทัด 94**: คอมเมนต์อธิบายการจำกัดช่วงคะแนน
- **บรรทัด 95**: `if (quality < 0.0) quality = 0.0;` - ป้องกันคะแนนติดลบ
- **บรรทัด 96**: `if (quality > 100.0) quality = 100.0;` - ป้องกันคะแนนเกิน 100%
- **บรรทัด 97**: คอมเมนต์ว่าง
- **บรรทัด 98**: `return quality;` - คืนค่าคะแนนคุณภาพที่คำนวณได้
- **บรรทัด 99**: `}` - ปิดฟังก์ชัน

### 6. ฟังก์ชัน Complementary Filter (บรรทัด 100-111)
```cpp
float applyComplementaryFilter(float accel_angle, float gyro_rate, float dt) {
    // การรวมข้อมูล gyroscope (ความถี่สูง, ความแม่นยำระยะสั้น)
    elevation_gyro = elevation_previous + gyro_rate * dt;
    
    // Complementary filter: รวม accelerometer (ความถี่ต่ำ) + gyroscope (ความถี่สูง)
    elevation_complementary = ALPHA * elevation_gyro + BETA * accel_angle;
    
    // อัปเดตค่าก่อนหน้าสำหรับการวนซ้ำครั้งถัดไป
    elevation_previous = elevation_complementary;
    
    return elevation_complementary;
}
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 100**: `float applyComplementaryFilter(...)` - ประกาศฟังก์ชันที่รับพารามิเตอร์ 3 ตัว (accel_angle, gyro_rate, dt) และคืนค่าเป็น float
- **บรรทัด 101**: คอมเมนต์อธิบายการรวมข้อมูล gyroscope
- **บรรทัด 102**: `elevation_gyro = elevation_previous + gyro_rate * dt;` - คำนวณมุมเงยจาก gyroscope โดยการรวม (integration) อัตราการหมุนคูณเวลา
- **บรรทัด 103**: คอมเมนต์ว่าง
- **บรรทัด 104**: คอมเมนต์อธิบาย Complementary filter
- **บรรทัด 105**: `elevation_complementary = ALPHA * elevation_gyro + BETA * accel_angle;` - รวมข้อมูลจาก gyroscope (98%) และ accelerometer (2%) เพื่อได้ผลลัพธ์ที่แม่นยำ
- **บรรทัด 106**: คอมเมนต์ว่าง
- **บรรทัด 107**: คอมเมนต์อธิบายการอัปเดตค่าก่อนหน้า
- **บรรทัด 108**: `elevation_previous = elevation_complementary;` - เก็บค่าปัจจุบันเพื่อใช้ในการคำนวณครั้งถัดไป
- **บรรทัด 109**: คอมเมนต์ว่าง
- **บรรทัด 110**: `return elevation_complementary;` - คืนค่ามุมเงยที่คำนวณได้จาก complementary filter
- **บรรทัด 111**: `}` - ปิดฟังก์ชัน

### 7. ตัวแปร BLE และข้อมูล (บรรทัด 113-135)
```cpp
// ตัวแปร BLE
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ตัวแปรข้อมูล
float distance = 0.0;  // ระยะทางจริงจาก PTFS sensor
int currentMode = 1;
unsigned long lastSendTime = 0;
const long sendInterval = 1000;  // เพิ่มจาก 500ms เป็น 1000ms เพื่อความเสถียร

// เพิ่มตัวแปรสำหรับการจัดการข้อผิดพลาด
bool mpu6050Healthy = false;
bool ptfsHealthy = false;
bool bleHealthy = false;
unsigned long lastMPU6050Check = 0;
unsigned long lastPTFSCheck = 0;
unsigned long lastBLECheck = 0;
const unsigned long healthCheckInterval = 5000; // ตรวจสอบทุก 5 วินาที
int errorCount = 0;
const int maxErrorCount = 10; // จำนวนข้อผิดพลาดสูงสุดก่อนรีเซ็ต
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 113**: คอมเมนต์หัวข้อสำหรับตัวแปร BLE
- **บรรทัด 114**: `BLEServer* pServer = NULL;` - ตัวแปร pointer สำหรับ BLE Server (เริ่มต้นเป็น NULL)
- **บรรทัด 115**: `BLECharacteristic* pCharacteristic = NULL;` - ตัวแปร pointer สำหรับ BLE Characteristic (เริ่มต้นเป็น NULL)
- **บรรทัด 116**: `bool deviceConnected = false;` - ตัวแปรบอกว่ามีอุปกรณ์เชื่อมต่อ BLE หรือไม่
- **บรรทัด 117**: `bool oldDeviceConnected = false;` - ตัวแปรเก็บสถานะการเชื่อมต่อก่อนหน้า
- **บรรทัด 118**: คอมเมนต์ว่างเพื่อแยกส่วน
- **บรรทัด 119**: คอมเมนต์หัวข้อสำหรับตัวแปรข้อมูล
- **บรรทัด 120**: `float distance = 0.0;` - ตัวแปรเก็บระยะทางจาก PTFS sensor (หน่วยเมตร)
- **บรรทัด 121**: `int currentMode = 1;` - ตัวแปรเก็บโหมดการทำงานปัจจุบัน (เริ่มต้นที่โหมด 1)
- **บรรทัด 122**: `unsigned long lastSendTime = 0;` - ตัวแปรเก็บเวลาที่ส่งข้อมูล BLE ครั้งล่าสุด
- **บรรทัด 123**: `const long sendInterval = 1000;` - ค่าคงที่สำหรับช่วงเวลาระหว่างการส่งข้อมูล BLE (1000ms = 1วินาที)
- **บรรทัด 124**: คอมเมนต์อธิบายการเพิ่มตัวแปรสำหรับการจัดการข้อผิดพลาด
- **บรรทัด 125**: `bool mpu6050Healthy = false;` - ตัวแปรบอกว่า MPU6050 ทำงานปกติหรือไม่
- **บรรทัด 126**: `bool ptfsHealthy = false;` - ตัวแปรบอกว่า PTFS sensor ทำงานปกติหรือไม่
- **บรรทัด 127**: `bool bleHealthy = false;` - ตัวแปรบอกว่า BLE ทำงานปกติหรือไม่
- **บรรทัด 128**: `unsigned long lastMPU6050Check = 0;` - ตัวแปรเก็บเวลาที่ตรวจสอบ MPU6050 ครั้งล่าสุด
- **บรรทัด 129**: `unsigned long lastPTFSCheck = 0;` - ตัวแปรเก็บเวลาที่ตรวจสอบ PTFS ครั้งล่าสุด
- **บรรทัด 130**: `unsigned long lastBLECheck = 0;` - ตัวแปรเก็บเวลาที่ตรวจสอบ BLE ครั้งล่าสุด
- **บรรทัด 131**: `const unsigned long healthCheckInterval = 5000;` - ค่าคงที่สำหรับช่วงเวลาระหว่างการตรวจสอบสุขภาพระบบ (5000ms = 5วินาที)
- **บรรทัด 132**: `int errorCount = 0;` - ตัวแปรนับจำนวนข้อผิดพลาดที่เกิดขึ้น
- **บรรทัด 133**: `const int maxErrorCount = 10;` - ค่าคงที่สำหรับจำนวนข้อผิดพลาดสูงสุดก่อนรีเซ็ตระบบ

### 8. ตัวแปร PTFS (บรรทัด 136-151)
```cpp
// ตัวแปร PTFS
uint8_t ptfsBuffer[200];
int ptfsBufferIndex = 0;
bool ptfsMeasurementActive = false;
bool loopbackDetected = false;  // เพิ่มตัวแปรตรวจจับ loopback
unsigned long lastPTFSCommand = 0;
unsigned long lastPTFSAnalysis = 0;
unsigned long lastPTFSDisplayTime = 0;
const unsigned long ptfsCommandInterval = 2000;  // เพิ่มเป็น 2 วินาที เพื่อความเสถียร
const unsigned long ptfsAnalysisInterval = 1000;  // เพิ่มเป็น 1 วินาที เพื่อความเสถียร

// ตัวกรองระยะทาง (เพิ่มขนาดเพื่อความเสถียร)
#define DISTANCE_FILTER_SIZE 30  // เพิ่มจาก 20 เป็น 30
float distanceHistory[DISTANCE_FILTER_SIZE] = {0};
int distanceFilterIndex = 0;
bool distanceFilterFull = false;
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 136**: คอมเมนต์หัวข้อสำหรับตัวแปร PTFS
- **บรรทัด 137**: `uint8_t ptfsBuffer[200];` - อาร์เรย์ขนาด 200 bytes สำหรับเก็บข้อมูลที่รับจาก PTFS sensor
- **บรรทัด 138**: `int ptfsBufferIndex = 0;` - ตัวแปรเก็บตำแหน่งปัจจุบันในบัฟเฟอร์
- **บรรทัด 139**: `bool ptfsMeasurementActive = false;` - ตัวแปรบอกว่า PTFS กำลังทำงานหรือไม่
- **บรรทัด 140**: `bool loopbackDetected = false;` - ตัวแปรตรวจจับการส่งข้อมูลกลับมาที่ตัวเอง (loopback)
- **บรรทัด 141**: `unsigned long lastPTFSCommand = 0;` - ตัวแปรเก็บเวลาที่ส่งคำสั่งไปยัง PTFS ครั้งล่าสุด
- **บรรทัด 142**: `unsigned long lastPTFSAnalysis = 0;` - ตัวแปรเก็บเวลาที่วิเคราะห์ข้อมูล PTFS ครั้งล่าสุด
- **บรรทัด 143**: `unsigned long lastPTFSDisplayTime = 0;` - ตัวแปรเก็บเวลาที่แสดงผลข้อมูล PTFS ครั้งล่าสุด
- **บรรทัด 144**: `const unsigned long ptfsCommandInterval = 2000;` - ค่าคงที่สำหรับช่วงเวลาระหว่างการส่งคำสั่ง PTFS (2000ms = 2วินาที)
- **บรรทัด 145**: `const unsigned long ptfsAnalysisInterval = 1000;` - ค่าคงที่สำหรับช่วงเวลาระหว่างการวิเคราะห์ข้อมูล PTFS (1000ms = 1วินาที)
- **บรรทัด 146**: คอมเมนต์ว่างเพื่อแยกส่วน
- **บรรทัด 147**: คอมเมนต์อธิบายตัวกรองระยะทาง
- **บรรทัด 148**: `#define DISTANCE_FILTER_SIZE 30` - กำหนดขนาดของตัวกรองระยะทางเป็น 30 samples
- **บรรทัด 149**: `float distanceHistory[DISTANCE_FILTER_SIZE] = {0};` - อาร์เรย์เก็บประวัติระยะทาง 30 ค่า (เริ่มต้นเป็น 0)
- **บรรทัด 150**: `int distanceFilterIndex = 0;` - ตัวแปรเก็บตำแหน่งปัจจุบันในตัวกรองระยะทาง
- **บรรทัด 151**: `bool distanceFilterFull = false;` - ตัวแปรบอกว่าตัวกรองระยะทางเต็มหรือไม่

### 9. BLE Callbacks (บรรทัด 153-180)
```cpp
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
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 153**: คอมเมนต์หัวข้อสำหรับ BLE Callbacks
- **บรรทัด 154**: `class MyServerCallbacks: public BLEServerCallbacks {` - สร้างคลาสสำหรับจัดการเหตุการณ์ BLE Server
- **บรรทัด 155**: `void onConnect(BLEServer* pServer) {` - ฟังก์ชันที่เรียกเมื่อมีอุปกรณ์เชื่อมต่อ BLE
- **บรรทัด 156**: `deviceConnected = true;` - ตั้งค่าสถานะการเชื่อมต่อเป็น true
- **บรรทัด 157**: `Serial.println("🔵 อุปกรณ์เชื่อมต่อ!");` - แสดงข้อความเมื่อมีอุปกรณ์เชื่อมต่อ
- **บรรทัด 158**: `};` - ปิดฟังก์ชัน onConnect
- **บรรทัด 159**: คอมเมนต์ว่าง
- **บรรทัด 160**: `void onDisconnect(BLEServer* pServer) {` - ฟังก์ชันที่เรียกเมื่ออุปกรณ์ตัดการเชื่อมต่อ BLE
- **บรรทัด 161**: `deviceConnected = false;` - ตั้งค่าสถานะการเชื่อมต่อเป็น false
- **บรรทัด 162**: `Serial.println("🔴 อุปกรณ์ตัดการเชื่อมต่อ!");` - แสดงข้อความเมื่ออุปกรณ์ตัดการเชื่อมต่อ
- **บรรทัด 163**: `}` - ปิดฟังก์ชัน onDisconnect
- **บรรทัด 164**: `};` - ปิดคลาส MyServerCallbacks
- **บรรทัด 165**: คอมเมนต์ว่าง
- **บรรทัด 166**: `class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {` - สร้างคลาสสำหรับจัดการเหตุการณ์ BLE Characteristic
- **บรรทัด 167**: `void onWrite(BLECharacteristic *pCharacteristic) {` - ฟังก์ชันที่เรียกเมื่อมีข้อมูลเขียนมาที่ BLE Characteristic
- **บรรทัด 168**: `uint8_t* data = pCharacteristic->getData();` - ดึงข้อมูลที่เขียนมา
- **บรรทัด 169**: `size_t len = pCharacteristic->getLength();` - ดึงความยาวของข้อมูล
- **บรรทัด 170**: `String value = "";` - สร้างตัวแปร String สำหรับเก็บข้อมูล
- **บรรทัด 171**: `for (int i = 0; i < len; i++) {` - วนลูปเพื่อแปลงข้อมูลเป็น String
- **บรรทัด 172**: `value += (char)data[i];` - แปลงข้อมูลแต่ละ byte เป็น char และเพิ่มเข้า String
- **บรรทัด 173**: `}` - ปิด for loop
- **บรรทัด 174**: `Serial.print("รับคำสั่ง: ");` - แสดงข้อความ "รับคำสั่ง: "
- **บรรทัด 175**: `Serial.println(value);` - แสดงค่าที่รับมา
- **บรรทัด 176**: คอมเมนต์ว่าง
- **บรรทัด 177**: `if (value == "MODE1") currentMode = 1;` - ตรวจสอบว่าค่าที่รับมาเป็น "MODE1" หรือไม่ และตั้งค่า currentMode เป็น 1
- **บรรทัด 178**: `else if (value == "MODE2") currentMode = 2;` - ตรวจสอบว่าค่าที่รับมาเป็น "MODE2" หรือไม่ และตั้งค่า currentMode เป็น 2
- **บรรทัด 179**: `}` - ปิดฟังก์ชัน onWrite
- **บรรทัด 180**: `};` - ปิดคลาส MyCharacteristicCallbacks

### 10. ฟังก์ชันตรวจสอบสถานะระบบ (บรรทัด 182-248)
```cpp
// ================ System Health Monitoring ================
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
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 182**: คอมเมนต์หัวข้อสำหรับ System Health Monitoring
- **บรรทัด 183**: `bool checkMPU6050Health() {` - ฟังก์ชันตรวจสอบสถานะ MPU6050
- **บรรทัด 184**: `if (mpu.testConnection()) {` - ตรวจสอบการเชื่อมต่อกับ MPU6050
- **บรรทัด 185**: `mpu6050Healthy = true;` - ตั้งค่าสถานะ MPU6050 เป็นปกติ
- **บรรทัด 186**: `return true;` - คืนค่า true เมื่อเชื่อมต่อได้
- **บรรทัด 187**: `} else {` - กรณีที่เชื่อมต่อไม่ได้
- **บรรทัด 188**: `mpu6050Healthy = false;` - ตั้งค่าสถานะ MPU6050 เป็นผิดปกติ
- **บรรทัด 189**: `Serial.println("❌ MPU6050 Health Check Failed!");` - แสดงข้อความข้อผิดพลาด
- **บรรทัด 190**: `return false;` - คืนค่า false เมื่อเชื่อมต่อไม่ได้
- **บรรทัด 191**: `}` - ปิด else
- **บรรทัด 192**: `}` - ปิดฟังก์ชัน checkMPU6050Health
- **บรรทัด 193**: คอมเมนต์ว่าง
- **บรรทัด 194**: `bool checkPTFSHealth() {` - ฟังก์ชันตรวจสอบสถานะ PTFS
- **บรรทัด 195**: คอมเมนต์อธิบายการตรวจสอบการตอบสนองจาก PTFS
- **บรรทัด 196**: `if (millis() - lastPTFSAnalysis < 10000) {` - ตรวจสอบว่ามีการวิเคราะห์ข้อมูล PTFS ใน 10 วินาทีที่ผ่านมาหรือไม่
- **บรรทัด 197**: `ptfsHealthy = true;` - ตั้งค่าสถานะ PTFS เป็นปกติ
- **บรรทัด 198**: `return true;` - คืนค่า true เมื่อ PTFS ทำงานปกติ
- **บรรทัด 199**: `} else {` - กรณีที่ PTFS ไม่ตอบสนอง
- **บรรทัด 200**: `ptfsHealthy = false;` - ตั้งค่าสถานะ PTFS เป็นผิดปกติ
- **บรรทัด 201**: `Serial.println("⚠️ PTFS Health Check Failed - No Response!");` - แสดงข้อความข้อผิดพลาด
- **บรรทัด 202**: `return false;` - คืนค่า false เมื่อ PTFS ไม่ตอบสนอง
- **บรรทัด 203**: `}` - ปิด else
- **บรรทัด 204**: `}` - ปิดฟังก์ชัน checkPTFSHealth
- **บรรทัด 205**: คอมเมนต์ว่าง
- **บรรทัด 206**: `bool checkBLEHealth() {` - ฟังก์ชันตรวจสอบสถานะ BLE
- **บรรทัด 207**: `if (pServer && pCharacteristic) {` - ตรวจสอบว่า BLE Server และ Characteristic ถูกสร้างแล้วหรือไม่
- **บรรทัด 208**: `bleHealthy = true;` - ตั้งค่าสถานะ BLE เป็นปกติ
- **บรรทัด 209**: `return true;` - คืนค่า true เมื่อ BLE ทำงานปกติ
- **บรรทัด 210**: `} else {` - กรณีที่ BLE ไม่ทำงาน
- **บรรทัด 211**: `bleHealthy = false;` - ตั้งค่าสถานะ BLE เป็นผิดปกติ
- **บรรทัด 212**: `Serial.println("❌ BLE Health Check Failed!");` - แสดงข้อความข้อผิดพลาด
- **บรรทัด 213**: `return false;` - คืนค่า false เมื่อ BLE ไม่ทำงาน
- **บรรทัด 214**: `}` - ปิด else
- **บรรทัด 215**: `}` - ปิดฟังก์ชัน checkBLEHealth
- **บรรทัด 216**: คอมเมนต์ว่าง
- **บรรทัด 217**: `void performHealthCheck() {` - ฟังก์ชันตรวจสอบสุขภาพระบบทั้งหมด
- **บรรทัด 218**: `static unsigned long lastHealthCheck = 0;` - ตัวแปรเก็บเวลาที่ตรวจสอบสุขภาพครั้งล่าสุด
- **บรรทัด 219**: คอมเมนต์ว่าง
- **บรรทัด 220**: `if (millis() - lastHealthCheck >= healthCheckInterval) {` - ตรวจสอบว่าถึงเวลาตรวจสอบสุขภาพหรือไม่
- **บรรทัด 221**: `bool mpuOK = checkMPU6050Health();` - ตรวจสอบสถานะ MPU6050
- **บรรทัด 222**: `bool ptfsOK = checkPTFSHealth();` - ตรวจสอบสถานะ PTFS
- **บรรทัด 223**: `bool bleOK = checkBLEHealth();` - ตรวจสอบสถานะ BLE
- **บรรทัด 224**: คอมเมนต์ว่าง
- **บรรทัด 225**: คอมเมนต์อธิบายการนับข้อผิดพลาด
- **บรรทัด 226**: `if (!mpuOK || !ptfsOK || !bleOK) {` - ตรวจสอบว่ามีข้อผิดพลาดในระบบหรือไม่
- **บรรทัด 227**: `errorCount++;` - เพิ่มจำนวนข้อผิดพลาด
- **บรรทัด 228**: `Serial.printf("⚠️ Health Check Failed! Error Count: %d/%d\n", errorCount, maxErrorCount);` - แสดงจำนวนข้อผิดพลาด
- **บรรทัด 229**: คอมเมนต์ว่าง
- **บรรทัด 230**: คอมเมนต์อธิบายการรีเซ็ตระบบ
- **บรรทัด 231**: `if (errorCount >= maxErrorCount) {` - ตรวจสอบว่าจำนวนข้อผิดพลาดเกินขีดจำกัดหรือไม่
- **บรรทัด 232**: `Serial.println("🔄 Too Many Errors - Restarting System...");` - แสดงข้อความการรีเซ็ตระบบ
- **บรรทัด 233**: `ESP.restart();` - รีเซ็ต ESP32
- **บรรทัด 234**: `}` - ปิด if
- **บรรทัด 235**: `} else {` - กรณีที่ระบบทำงานปกติ
- **บรรทัด 236**: `errorCount = 0;` - รีเซ็ตตัวนับข้อผิดพลาด
- **บรรทัด 237**: คอมเมนต์อธิบายการรีเซ็ตตัวนับ
- **บรรทัด 238**: `}` - ปิด else
- **บรรทัด 239**: คอมเมนต์ว่าง
- **บรรทัด 240**: คอมเมนต์อธิบายการแสดงสถานะหน่วยความจำ
- **บรรทัด 241**: `Serial.printf("💾 Free Heap: %d bytes\n", ESP.getFreeHeap());` - แสดงหน่วยความจำว่าง
- **บรรทัด 242**: `if (ESP.getFreeHeap() < 10000) {` - ตรวจสอบว่าหน่วยความจำว่างน้อยกว่า 10KB หรือไม่
- **บรรทัด 243**: `Serial.println("⚠️ Low Memory Warning!");` - แสดงคำเตือนหน่วยความจำต่ำ
- **บรรทัด 244**: `}` - ปิด if
- **บรรทัด 245**: คอมเมนต์ว่าง
- **บรรทัด 246**: `lastHealthCheck = millis();` - อัปเดตเวลาตรวจสอบสุขภาพครั้งล่าสุด
- **บรรทัด 247**: `}` - ปิด if
- **บรรทัด 248**: `}` - ปิดฟังก์ชัน performHealthCheck

### 11. ฟังก์ชัน PTFS (บรรทัด 250-604)
```cpp
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
}

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
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 250**: คอมเมนต์หัวข้อสำหรับ PTFS Functions
- **บรรทัด 251**: `void sendPTFSCommand(uint8_t* cmd, uint8_t length) {` - ฟังก์ชันส่งคำสั่งไปยัง PTFS sensor
- **บรรทัด 252**: `for (int i = 0; i < length; i++) {` - วนลูปส่งข้อมูลแต่ละ byte
- **บรรทัด 253**: `Serial2.write(cmd[i]);` - ส่งข้อมูลแต่ละ byte ผ่าน UART2
- **บรรทัด 254**: `delayMicroseconds(100);` - หน่วงเวลา 100 ไมโครวินาทีระหว่างการส่ง
- **บรรทัด 255**: `}` - ปิด for loop
- **บรรทัด 256**: `Serial2.flush();` - รอให้ข้อมูลส่งเสร็จสิ้น
- **บรรทัด 257**: `}` - ปิดฟังก์ชัน sendPTFSCommand
- **บรรทัด 258**: คอมเมนต์ว่าง
- **บรรทัด 259**: `void startPTFSMeasurement() {` - ฟังก์ชันเริ่มการวัดระยะทาง PTFS
- **บรรทัด 260**: `if (!ptfsMeasurementActive) {` - ตรวจสอบว่า PTFS เปิดใช้งานหรือไม่
- **บรรทัด 261**: `return;` - ออกจากฟังก์ชันถ้า PTFS ไม่เปิดใช้งาน
- **บรรทัด 262**: `}` - ปิด if
- **บรรทัด 263**: คอมเมนต์ว่าง
- **บรรทัด 264**: คอมเมนต์อธิบายการส่งคำสั่ง PTFS Protocol
- **บรรทัด 265**: `uint8_t startCmd[] = {` - สร้างอาร์เรย์คำสั่งเริ่มการวัด
- **บรรทัด 266**: `0xFA,` - MsgType (ประเภทข้อความ)
- **บรรทัด 267**: `0x01,` - MsgCode (รหัสข้อความ - Start)
- **บรรทัด 268**: `0x00,` - BrdId (รหัสบอร์ด - broadcast)
- **บรรทัด 269**: `0x04,` - PayLoadLen (ความยาวข้อมูล)
- **บรรทัด 270**: `0x01, 0x00,` - MeaType (ประเภทการวัด - start measurement)
- **บรรทัด 271**: `0x00, 0x00,` - MeaTimes (จำนวนครั้งการวัด - unlimited)
- **บรรทัด 272**: `0x00` - CRC (เริ่มต้นเป็น 0)
- **บรรทัด 273**: `};` - ปิดอาร์เรย์
- **บรรทัด 274**: คอมเมนต์ว่าง
- **บรรทัด 275**: คอมเมนต์อธิบายการคำนวณ CRC
- **บรรทัด 276**: `uint8_t crc = 0;` - ตัวแปรเก็บค่า CRC
- **บรรทัด 277**: `for (int i = 0; i < 8; i++) {` - วนลูปคำนวณ CRC จาก 8 bytes แรก
- **บรรทัด 278**: `crc += startCmd[i];` - รวมค่าแต่ละ byte เข้า CRC
- **บรรทัด 279**: `}` - ปิด for loop
- **บรรทัด 280**: `startCmd[8] = crc & 0xFF;` - เก็บค่า CRC ในตำแหน่งที่ 8
- **บรรทัด 281**: คอมเมนต์ว่าง
- **บรรทัด 282**: คอมเมนต์อธิบายการส่งคำสั่ง
- **บรรทัด 283**: `sendPTFSCommand(startCmd, 9);` - ส่งคำสั่ง 9 bytes ไปยัง PTFS
- **บรรทัด 284**: `}` - ปิดฟังก์ชัน startPTFSMeasurement
- **บรรทัด 285**: คอมเมนต์ว่าง
- **บรรทัด 286**: `void addDistanceToFilter(float value) {` - ฟังก์ชันเพิ่มค่าระยะทางในตัวกรอง
- **บรรทัด 287**: `distanceHistory[distanceFilterIndex] = value;` - เก็บค่าระยะทางในตำแหน่งปัจจุบัน
- **บรรทัด 288**: `distanceFilterIndex = (distanceFilterIndex + 1) % DISTANCE_FILTER_SIZE;` - เลื่อนตำแหน่งถัดไป (วนกลับเมื่อเต็ม)
- **บรรทัด 289**: คอมเมนต์ว่าง
- **บรรทัด 290**: `if (!distanceFilterFull && distanceFilterIndex == 0) {` - ตรวจสอบว่าตัวกรองเต็มหรือไม่
- **บรรทัด 291**: `distanceFilterFull = true;` - ตั้งค่าตัวกรองเป็นเต็ม
- **บรรทัด 292**: `}` - ปิด if
- **บรรทัด 293**: `}` - ปิดฟังก์ชัน addDistanceToFilter
- **บรรทัด 294**: คอมเมนต์ว่าง
- **บรรทัด 295**: `void clearDistanceFilter() {` - ฟังก์ชันล้างตัวกรองระยะทาง
- **บรรทัด 296**: `for (int i = 0; i < DISTANCE_FILTER_SIZE; i++) {` - วนลูปล้างทุกค่าในตัวกรอง
- **บรรทัด 297**: `distanceHistory[i] = 0.0;` - ตั้งค่าแต่ละตำแหน่งเป็น 0
- **บรรทัด 298**: `}` - ปิด for loop
- **บรรทัด 299**: `distanceFilterIndex = 0;` - รีเซ็ตตำแหน่งปัจจุบัน
- **บรรทัด 300**: `distanceFilterFull = false;` - ตั้งค่าตัวกรองเป็นไม่เต็ม
- **บรรทัด 301**: `}` - ปิดฟังก์ชัน clearDistanceFilter
- **บรรทัด 302**: คอมเมนต์ว่าง
- **บรรทัด 303**: `float getFilteredDistance() {` - ฟังก์ชันคำนวณค่าระยะทางที่กรองแล้ว
- **บรรทัด 304**: `if (!distanceFilterFull && distanceFilterIndex == 0) return 0.0;` - คืนค่า 0 ถ้าตัวกรองยังว่าง
- **บรรทัด 305**: คอมเมนต์ว่าง
- **บรรทัด 306**: `float sum = 0;` - ตัวแปรเก็บผลรวม
- **บรรทัด 307**: `int count = distanceFilterFull ? DISTANCE_FILTER_SIZE : distanceFilterIndex;` - กำหนดจำนวนค่าที่จะคำนวณ
- **บรรทัด 308**: คอมเมนต์ว่าง
- **บรรทัด 309**: `for (int i = 0; i < count; i++) {` - วนลูปคำนวณผลรวม
- **บรรทัด 310**: `sum += distanceHistory[i];` - รวมค่าแต่ละตำแหน่ง
- **บรรทัด 311**: `}` - ปิด for loop
- **บรรทัด 312**: คอมเมนต์ว่าง
- **บรรทัด 313**: `return (count > 0) ? sum / count : 0.0;` - คืนค่าเฉลี่ยหรือ 0 ถ้าไม่มีข้อมูล
- **บรรทัด 314**: `}` - ปิดฟังก์ชัน getFilteredDistance
- **บรรทัด 315**: คอมเมนต์ว่าง
- **บรรทัด 316**: `void addDistance(uint16_t* distances, int* counts, int* foundCount, uint16_t value, int maxSize) {` - ฟังก์ชันเพิ่มค่าระยะทางในอาร์เรย์
- **บรรทัด 317**: คอมเมนต์อธิบายการหาค่าที่มีอยู่แล้ว
- **บรรทัด 318**: `for (int i = 0; i < *foundCount; i++) {` - วนลูปหาค่าที่มีอยู่แล้ว
- **บรรทัด 319**: `if (distances[i] == value) {` - ตรวจสอบว่าค่าซ้ำหรือไม่
- **บรรทัด 320**: `counts[i]++;` - เพิ่มจำนวนครั้งที่พบ
- **บรรทัด 321**: `return;` - ออกจากฟังก์ชัน
- **บรรทัด 322**: `}` - ปิด if
- **บรรทัด 323**: `}` - ปิด for loop
- **บรรทัด 324**: คอมเมนต์ว่าง
- **บรรทัด 325**: คอมเมนต์อธิบายการเพิ่มค่าใหม่
- **บรรทัด 326**: `if (*foundCount < maxSize) {` - ตรวจสอบว่ายังมีที่ว่างหรือไม่
- **บรรทัด 327**: `distances[*foundCount] = value;` - เก็บค่าใหม่
- **บรรทัด 328**: `counts[*foundCount] = 1;` - ตั้งจำนวนครั้งที่พบเป็น 1
- **บรรทัด 329**: `(*foundCount)++;` - เพิ่มจำนวนค่าที่พบ
- **บรรทัด 330**: `}` - ปิด if
- **บรรทัด 331**: `}` - ปิดฟังก์ชัน addDistance

### 12. ฟังก์ชันประมวลผลข้อมูล PTFS (บรรทัด 338-567)
```cpp
void processPTFSData(uint8_t* data, uint8_t length) {
    // เช็คว่าเป็น loopback data หรือไม่ (ปรับปรุงให้เสถียรขึ้น)
    if (length > 10 && data[0] == 0xFA) {
        return; // กรองเฉพาะเมื่อมีข้อมูลมากแล้ว
    }

    // ลองวิเคราะห์แม้ข้อมูลน้อย
    if (length < 3) return; // เพิ่มจาก 1 เป็น 3
    
    // ใช้ Map/Counter เล็กๆ สำหรับค่าที่ถูกต้อง
    const int maxDistances = 20;
    uint16_t distances[maxDistances];
    int distanceCount[maxDistances];
    int foundDistances = 0;
    
    // PTFS Official Protocol - Table 5-2 Measurement Report
    // Format: 0xFB 0x03 BrdId 0x04 DataValidInd Distance_L Distance_H CRC
    
    // Pattern 1: PTFS Standard Protocol (ตามคู่มือ)
    for (int i = 0; i <= length - 8; i++) {
        if (data[i] == 0xFB && data[i+1] == 0x03 && data[i+3] == 0x04) {
            uint8_t dataValid = data[i+4];  // DataValidInd (1=valid, 0=invalid)
            
            if (dataValid == 1) {  // ข้อมูลถูกต้อง
                // Distance in Little Endian format (dm)
                uint16_t dm_value = data[i+6] | (data[i+7] << 8);
                
                // ตรวจสอบข้อมูลที่ถูกต้อง
                if (dm_value >= 30 && dm_value <= 10000) {  // 3m-1000m
                    addDistance(distances, distanceCount, &foundDistances, dm_value, maxDistances);
                } else if (dm_value == 0) {
                    // ระยะทาง 0m = ไม่มีวัตถุในระยะวัด
                } else {
                    // ระยะทางไม่ถูกต้อง
                }
            } else {
                // ข้อมูลไม่ถูกต้อง
            }
        }
    }
    
    // หาค่าที่มีความถี่สูงสุด และมีความน่าเชื่อถือ
    int bestDm = 0;
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
    
    // แสดงผลตามที่ผู้ใช้ต้องการ (พร้อม Moving Average Filter)
    if (bestDm > 0) {
        // ตรวจสอบระยะทางที่ถูกต้อง
        if (bestDm < 30) { // น้อยกว่า 3.0m
            // ล้าง Filter และตั้งค่าระยะทางเป็น 0 เมื่อน้อยกว่า 3m
            clearDistanceFilter();
            distance = 0.0;
        } else {
            // เพิ่มค่าใน Moving Average Filter
            float meters = bestDm / 10.0;
            
            // ตรวจสอบค่าที่สมเหตุสมผล
            if (meters > 0 && meters < 1000) {
                addDistanceToFilter(meters);
                distance = getFilteredDistance();
            } else {
                // ล้าง Filter และตั้งค่าระยะทางเป็น 0 เมื่อไม่สมเหตุสมผล
                clearDistanceFilter();
                distance = 0.0;
            }
        }
    } else {
        // ไม่พบข้อมูลระยะทางที่ถูกต้อง - ตั้งเป็น 0m
        clearDistanceFilter();
        distance = 0.0;
    }
}
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 338**: `void processPTFSData(uint8_t* data, uint8_t length) {` - ฟังก์ชันประมวลผลข้อมูลที่รับจาก PTFS sensor
- **บรรทัด 339**: คอมเมนต์อธิบายการตรวจสอบ loopback data
- **บรรทัด 340**: `if (length > 10 && data[0] == 0xFA) {` - ตรวจสอบว่าข้อมูลเป็น loopback หรือไม่
- **บรรทัด 341**: `return;` - ออกจากฟังก์ชันถ้าเป็น loopback
- **บรรทัด 342**: `}` - ปิด if
- **บรรทัด 343**: คอมเมนต์ว่าง
- **บรรทัด 344**: คอมเมนต์อธิบายการวิเคราะห์ข้อมูล
- **บรรทัด 345**: `if (length < 3) return;` - ออกจากฟังก์ชันถ้าข้อมูลน้อยเกินไป
- **บรรทัด 346**: คอมเมนต์อธิบายการเพิ่มจาก 1 เป็น 3
- **บรรทัด 347**: คอมเมนต์ว่าง
- **บรรทัด 348**: คอมเมนต์อธิบายการใช้ Map/Counter
- **บรรทัด 349**: `const int maxDistances = 20;` - กำหนดจำนวนระยะทางสูงสุดที่เก็บได้
- **บรรทัด 350**: `uint16_t distances[maxDistances];` - อาร์เรย์เก็บค่าระยะทางที่พบ
- **บรรทัด 351**: `int distanceCount[maxDistances];` - อาร์เรย์เก็บจำนวนครั้งที่พบค่าระยะทางแต่ละค่า
- **บรรทัด 352**: `int foundDistances = 0;` - ตัวแปรนับจำนวนระยะทางที่พบ
- **บรรทัด 353**: คอมเมนต์ว่าง
- **บรรทัด 354**: คอมเมนต์อธิบาย PTFS Official Protocol
- **บรรทัด 355**: คอมเมนต์อธิบายรูปแบบข้อมูล
- **บรรทัด 356**: คอมเมนต์ว่าง
- **บรรทัด 357**: คอมเมนต์อธิบาย Pattern 1
- **บรรทัด 358**: `for (int i = 0; i <= length - 8; i++) {` - วนลูปค้นหาข้อมูล PTFS Protocol
- **บรรทัด 359**: `if (data[i] == 0xFB && data[i+1] == 0x03 && data[i+3] == 0x04) {` - ตรวจสอบรูปแบบข้อมูล PTFS
- **บรรทัด 360**: `uint8_t dataValid = data[i+4];` - ดึงค่า DataValidInd (1=valid, 0=invalid)
- **บรรทัด 361**: คอมเมนต์อธิบาย DataValidInd
- **บรรทัด 362**: คอมเมนต์ว่าง
- **บรรทัด 363**: `if (dataValid == 1) {` - ตรวจสอบว่าข้อมูลถูกต้องหรือไม่
- **บรรทัด 364**: คอมเมนต์อธิบายข้อมูลถูกต้อง
- **บรรทัด 365**: คอมเมนต์อธิบาย Distance in Little Endian format
- **บรรทัด 366**: `uint16_t dm_value = data[i+6] | (data[i+7] << 8);` - รวมข้อมูลระยะทาง 2 bytes เป็น 16-bit
- **บรรทัด 367**: คอมเมนต์ว่าง
- **บรรทัด 368**: คอมเมนต์อธิบายการตรวจสอบข้อมูลที่ถูกต้อง
- **บรรทัด 369**: `if (dm_value >= 30 && dm_value <= 10000) {` - ตรวจสอบว่าระยะทางอยู่ในช่วง 3m-1000m หรือไม่
- **บรรทัด 370**: `addDistance(distances, distanceCount, &foundDistances, dm_value, maxDistances);` - เพิ่มค่าระยะทางในอาร์เรย์
- **บรรทัด 371**: `} else if (dm_value == 0) {` - ตรวจสอบว่าระยะทางเป็น 0m หรือไม่
- **บรรทัด 372**: คอมเมนต์อธิบายระยะทาง 0m
- **บรรทัด 373**: `} else {` - กรณีอื่นๆ
- **บรรทัด 374**: คอมเมนต์อธิบายระยะทางไม่ถูกต้อง
- **บรรทัด 375**: `}` - ปิด else
- **บรรทัด 376**: `} else {` - กรณีที่ข้อมูลไม่ถูกต้อง
- **บรรทัด 377**: คอมเมนต์อธิบายข้อมูลไม่ถูกต้อง
- **บรรทัด 378**: `}` - ปิด else
- **บรรทัด 379**: `}` - ปิด if
- **บรรทัด 380**: `}` - ปิด for loop
- **บรรทัด 381**: คอมเมนต์ว่าง
- **บรรทัด 382**: คอมเมนต์อธิบายการหาค่าที่มีความถี่สูงสุด
- **บรรทัด 383**: `int bestDm = 0;` - ตัวแปรเก็บค่าระยะทางที่ดีที่สุด
- **บรรทัด 384**: `int bestCount = 0;` - ตัวแปรเก็บจำนวนครั้งที่พบค่าดีที่สุด
- **บรรทัด 385**: `for (int i = 0; i < foundDistances; i++) {` - วนลูปหาค่าที่มีความถี่สูงสุด
- **บรรทัด 386**: `if (distanceCount[i] > bestCount && distanceCount[i] >= 5) {` - ตรวจสอบว่าค่านี้มีความถี่สูงกว่าและมากกว่า 5 ครั้ง
- **บรรทัด 387**: คอมเมนต์อธิบายการเพิ่มจาก 3 เป็น 5
- **บรรทัด 388**: `bestCount = distanceCount[i];` - อัปเดตจำนวนครั้งที่ดีที่สุด
- **บรรทัด 389**: `bestDm = distances[i];` - อัปเดตค่าระยะทางที่ดีที่สุด
- **บรรทัด 390**: `}` - ปิด if
- **บรรทัด 391**: `}` - ปิด for loop
- **บรรทัด 392**: คอมเมนต์ว่าง
- **บรรทัด 393**: คอมเมนต์อธิบายการหาค่าเฉลี่ย
- **บรรทัด 394**: `if (bestDm == 0 && foundDistances > 0) {` - ตรวจสอบว่าไม่พบค่าที่เชื่อถือได้แต่มีข้อมูลอยู่
- **บรรทัด 395**: `uint32_t sum = 0;` - ตัวแปรเก็บผลรวม
- **บรรทัด 396**: `int count = 0;` - ตัวแปรนับจำนวนค่าที่รวม
- **บรรทัด 397**: `uint16_t refValue = distances[0];` - ใช้ค่าแรกเป็นค่าอ้างอิง
- **บรรทัด 398**: คอมเมนต์ว่าง
- **บรรทัด 399**: คอมเมนต์อธิบายการรวมค่าที่ใกล้เคียงกัน
- **บรรทัด 400**: `for (int i = 0; i < foundDistances; i++) {` - วนลูปหาค่าที่ใกล้เคียงกัน
- **บรรทัด 401**: `if (abs((int)distances[i] - (int)refValue) <= 20) {` - ตรวจสอบว่าค่าต่างกันไม่เกิน 20dm (2m)
- **บรรทัด 402**: `sum += distances[i];` - รวมค่าเข้าไปในผลรวม
- **บรรทัด 403**: `count++;` - เพิ่มจำนวนค่าที่รวม
- **บรรทัด 404**: `}` - ปิด if
- **บรรทัด 405**: `}` - ปิด for loop
- **บรรทัด 406**: คอมเมนต์ว่าง
- **บรรทัด 407**: `if (count >= 2) {` - ตรวจสอบว่ามีค่าที่รวมอย่างน้อย 2 ค่า
- **บรรทัด 408**: `bestDm = sum / count;` - คำนวณค่าเฉลี่ย
- **บรรทัด 409**: คอมเมนต์อธิบายค่าเฉลี่ยที่เสถียร
- **บรรทัด 410**: `} else {` - กรณีที่มีค่าน้อยกว่า 2 ค่า
- **บรรทัด 411**: `bestDm = distances[0];` - ใช้ค่าแรก
- **บรรทัด 412**: คอมเมนต์อธิบายการใช้ค่าแรก
- **บรรทัด 413**: `}` - ปิด else
- **บรรทัด 414**: `}` - ปิด if
- **บรรทัด 415**: คอมเมนต์ว่าง
- **บรรทัด 416**: คอมเมนต์อธิบายการแสดงผล
- **บรรทัด 417**: `if (bestDm > 0) {` - ตรวจสอบว่าพบค่าระยะทางที่ถูกต้องหรือไม่
- **บรรทัด 418**: คอมเมนต์อธิบายการตรวจสอบระยะทางที่ถูกต้อง
- **บรรทัด 419**: `if (bestDm < 30) {` - ตรวจสอบว่าระยะทางน้อยกว่า 3.0m หรือไม่
- **บรรทัด 420**: คอมเมนต์อธิบายการล้าง Filter
- **บรรทัด 421**: `clearDistanceFilter();` - ล้างตัวกรองระยะทาง
- **บรรทัด 422**: `distance = 0.0;` - ตั้งค่าระยะทางเป็น 0
- **บรรทัด 423**: `} else {` - กรณีที่ระยะทางมากกว่าหรือเท่ากับ 3.0m
- **บรรทัด 424**: คอมเมนต์อธิบายการเพิ่มค่าใน Moving Average Filter
- **บรรทัด 425**: `float meters = bestDm / 10.0;` - แปลงค่าระยะทางจาก dm เป็น m
- **บรรทัด 426**: คอมเมนต์ว่าง
- **บรรทัด 427**: คอมเมนต์อธิบายการตรวจสอบค่าที่สมเหตุสมผล
- **บรรทัด 428**: `if (meters > 0 && meters < 1000) {` - ตรวจสอบว่าระยะทางอยู่ในช่วง 0-1000m หรือไม่
- **บรรทัด 429**: `addDistanceToFilter(meters);` - เพิ่มค่าระยะทางในตัวกรอง
- **บรรทัด 430**: `distance = getFilteredDistance();` - คำนวณค่าระยะทางที่กรองแล้ว
- **บรรทัด 431**: `} else {` - กรณีที่ระยะทางไม่อยู่ในช่วงที่กำหนด
- **บรรทัด 432**: คอมเมนต์อธิบายการล้าง Filter
- **บรรทัด 433**: `clearDistanceFilter();` - ล้างตัวกรองระยะทาง
- **บรรทัด 434**: `distance = 0.0;` - ตั้งค่าระยะทางเป็น 0
- **บรรทัด 435**: `}` - ปิด else
- **บรรทัด 436**: `}` - ปิด else
- **บรรทัด 437**: `} else {` - กรณีที่ไม่พบค่าระยะทางที่ถูกต้อง
- **บรรทัด 438**: คอมเมนต์อธิบายการไม่พบข้อมูลระยะทางที่ถูกต้อง
- **บรรทัด 439**: `clearDistanceFilter();` - ล้างตัวกรองระยะทาง
- **บรรทัด 440**: `distance = 0.0;` - ตั้งค่าระยะทางเป็น 0
- **บรรทัด 441**: `}` - ปิด else
- **บรรทัด 442**: `}` - ปิดฟังก์ชัน processPTFSData

### 13. ฟังก์ชันประมวลผลสตรีม PTFS (บรรทัด 569-604)
```cpp
void processPTFSStream() {
    // ปรับปรุงการจัดการหน่วยความจำ
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
        
        // ปรับปรุงการจัดการบัฟเฟอร์
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
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 569**: `void processPTFSStream() {` - ฟังก์ชันประมวลผลสตรีมข้อมูลจาก PTFS sensor
- **บรรทัด 570**: คอมเมนต์อธิบายการปรับปรุงการจัดการหน่วยความจำ
- **บรรทัด 571**: `static unsigned long lastBufferCleanup = 0;` - ตัวแปรเก็บเวลาที่ล้างบัฟเฟอร์ครั้งล่าสุด
- **บรรทัด 572**: คอมเมนต์ว่าง
- **บรรทัด 573**: คอมเมนต์อธิบายการล้างบัฟเฟอร์เป็นระยะ
- **บรรทัด 574**: `if (millis() - lastBufferCleanup >= 30000) {` - ตรวจสอบว่าถึงเวลาล้างบัฟเฟอร์หรือไม่ (30 วินาที)
- **บรรทัด 575**: คอมเมนต์อธิบายทุก 30 วินาที
- **บรรทัด 576**: `ptfsBufferIndex = 0;` - รีเซ็ตตำแหน่งบัฟเฟอร์
- **บรรทัด 577**: `lastBufferCleanup = millis();` - อัปเดตเวลาล้างบัฟเฟอร์ครั้งล่าสุด
- **บรรทัด 578**: `}` - ปิด if
- **บรรทัด 579**: คอมเมนต์ว่าง
- **บรรทัด 580**: `while (Serial2.available()) {` - วนลูปอ่านข้อมูลจาก UART2 ตราบใดที่มีข้อมูล
- **บรรทัด 581**: `uint8_t newByte = Serial2.read();` - อ่านข้อมูล 1 byte จาก UART2
- **บรรทัด 582**: คอมเมนต์ว่าง
- **บรรทัด 583**: คอมเมนต์อธิบายการลดการตรวจจับ loopback
- **บรรทัด 584**: `if (newByte == 0xFA && ptfsBufferIndex > 15) {` - ตรวจสอบว่าข้อมูลเป็น 0xFA และบัฟเฟอร์มีข้อมูลมากแล้วหรือไม่
- **บรรทัด 585**: คอมเมนต์อธิบายการล้างบัฟเฟอร์เฉพาะเมื่อมีข้อมูลมากแล้ว
- **บรรทัด 586**: `ptfsBufferIndex = 0;` - รีเซ็ตตำแหน่งบัฟเฟอร์
- **บรรทัด 587**: `continue;` - ข้ามการประมวลผลข้อมูลนี้
- **บรรทัด 588**: `}` - ปิด if
- **บรรทัด 589**: คอมเมนต์ว่าง
- **บรรทัด 590**: `ptfsBuffer[ptfsBufferIndex++] = newByte;` - เก็บข้อมูลในบัฟเฟอร์และเลื่อนตำแหน่งถัดไป
- **บรรทัด 591**: คอมเมนต์ว่าง
- **บรรทัด 592**: คอมเมนต์อธิบายการปรับปรุงการจัดการบัฟเฟอร์
- **บรรทัด 593**: `if (ptfsBufferIndex >= 200) {` - ตรวจสอบว่าบัฟเฟอร์เต็มหรือไม่
- **บรรทัด 594**: `ptfsBufferIndex = 0;` - รีเซ็ตตำแหน่งบัฟเฟอร์
- **บรรทัด 595**: คอมเมนต์อธิบายการรีเซ็ตบัฟเฟอร์เมื่อเต็ม
- **บรรทัด 596**: `Serial.println("⚠️ PTFS Buffer Overflow - Reset!");` - แสดงข้อความเตือนบัฟเฟอร์ล้น
- **บรรทัด 597**: `}` - ปิด if
- **บรรทัด 598**: `}` - ปิด while loop
- **บรรทัด 599**: คอมเมนต์ว่าง
- **บรรทัด 600**: คอมเมนต์อธิบายการวิเคราะห์ข้อมูลช้าลง
- **บรรทัด 601**: `if (ptfsBufferIndex >= 8) {` - ตรวจสอบว่าบัฟเฟอร์มีข้อมูลอย่างน้อย 8 bytes หรือไม่
- **บรรทัด 602**: คอมเมนต์อธิบายการเพิ่มจาก 2 เป็น 8
- **บรรทัด 603**: `processPTFSData(ptfsBuffer, ptfsBufferIndex);` - เรียกฟังก์ชันประมวลผลข้อมูล PTFS
- **บรรทัด 604**: `lastPTFSAnalysis = millis();` - อัปเดตเวลาวิเคราะห์ข้อมูล PTFS ครั้งล่าสุด
- **บรรทัด 605**: `ptfsBufferIndex = 0;` - รีเซ็ตตำแหน่งบัฟเฟอร์
- **บรรทัด 606**: `}` - ปิด if
- **บรรทัด 607**: `}` - ปิดฟังก์ชัน processPTFSStream

### 14. ฟังก์ชัน MPU6050 (บรรทัด 606-892)
```cpp
// ================ MPU6050 Functions ================
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
    
    // Professional calibration for both accelerometer and gyroscope
    ax_offset = ax_sum / samples;
    ay_offset = ay_sum / samples;
    az_offset = az_sum / samples - 16384; // Z-axis = 0 เมื่อวางราบ
    
    // Gyroscope calibration (should be 0 when stationary)
    gx_offset = gx_sum / samples;
    gy_offset = gy_sum / samples;
    gz_offset = gz_sum / samples;
    
    calibrated = true;
    Serial.println("✅ PROFESSIONAL MPU6050 Calibration Complete!");
}

void calculateElevationAngle() {
    // Professional Elevation Calculation with ±0.1° accuracy
    
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
    
    // Professional Elevation Calculation with Complementary Filter
    
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
    
    // ใช้ Complementary Filter แทน Low-pass + Moving Average
    // Gyroscope rate สำหรับ pitch (elevation) คือ Gy (Y-axis rotation)
    float gyro_rate = Gy; // °/s
    
    // ใช้ Complementary Filter สำหรับความแม่นยำ ±0.1°
    elevation_complementary = applyComplementaryFilter(elevation_raw, gyro_rate, DT);
    
    // จำกัดค่าผลลัพธ์สุดท้าย
    if (elevation_complementary > 90.0) elevation_complementary = 90.0;
    if (elevation_complementary < -90.0) elevation_complementary = -90.0;
}
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 606**: คอมเมนต์หัวข้อสำหรับ MPU6050 Functions
- **บรรทัด 607**: `void initializeAdvancedPitch() {` - ฟังก์ชันเริ่มต้นการคำนวณมุมเงยขั้นสูง
- **บรรทัด 608**: `Serial.println("🔧 Initializing Advanced Pitch Calculation...");` - แสดงข้อความเริ่มต้น
- **บรรทัด 609**: `Serial.println("   📐 Using Accelerometer + Gyroscope for accurate pitch");` - แสดงข้อความอธิบายการใช้เซ็นเซอร์
- **บรรทัด 610**: `Serial.println("   🎯 Range: -90° to +90° (Pitch/Elevation)");` - แสดงข้อความอธิบายช่วงมุม
- **บรรทัด 611**: `Serial.println("   ✅ Roll Independent - ไม่เปลี่ยนเมื่อเอียงซ้ายขวา");` - แสดงข้อความอธิบายความเสถียร
- **บรรทัด 612**: `}` - ปิดฟังก์ชัน initializeAdvancedPitch
- **บรรทัด 613**: คอมเมนต์ว่าง
- **บรรทัด 614**: `void calibrateMPU6050() {` - ฟังก์ชันคาลิเบรต MPU6050
- **บรรทัด 615**: `Serial.println("🔧 Starting PROFESSIONAL MPU6050 calibration for ±0.1° accuracy...");` - แสดงข้อความเริ่มต้นการคาลิเบรต
- **บรรทัด 616**: `Serial.println("   📐 Keep sensor FLAT and STILL for 30 seconds...");` - แสดงข้อความแนะนำการวางเซ็นเซอร์
- **บรรทัด 617**: `Serial.println("   🎯 This will set Elevation = 0° when sensor is level (แนวระดับสายตา)");` - แสดงข้อความอธิบายการตั้งค่า
- **บรรทัด 618**: `Serial.println("   ⚠️  Make sure sensor is perfectly horizontal (ขนานกับพื้นโลก)!");` - แสดงข้อความเตือนการวางเซ็นเซอร์
- **บรรทัด 619**: `Serial.println("   🎯 Enhanced calibration: 3,000 samples for maximum precision");` - แสดงข้อความอธิบายการคาลิเบรต
- **บรรทัด 620**: คอมเมนต์ว่าง
- **บรรทัด 621**: `delay(3000);` - หน่วงเวลา 3 วินาที
- **บรรทัด 622**: คอมเมนต์ว่าง
- **บรรทัด 623**: `long ax_sum = 0, ay_sum = 0, az_sum = 0;` - ตัวแปรเก็บผลรวมข้อมูล accelerometer
- **บรรทัด 624**: `long gx_sum = 0, gy_sum = 0, gz_sum = 0;` - ตัวแปรเก็บผลรวมข้อมูล gyroscope
- **บรรทัด 625**: `const int samples = 3000;` - กำหนดจำนวนตัวอย่าง 3,000 ครั้ง
- **บรรทัด 626**: คอมเมนต์อธิบายการเพิ่มเป็น 3,000 samples
- **บรรทัด 627**: คอมเมนต์ว่าง
- **บรรทัด 628**: `Serial.println("   🔄 Professional Calibrating... (30 seconds)");` - แสดงข้อความการคาลิเบรต
- **บรรทัด 629**: คอมเมนต์ว่าง
- **บรรทัด 630**: `for (int i = 0; i < samples; i++) {` - วนลูปคาลิเบรต 3,000 ครั้ง
- **บรรทัด 631**: `mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);` - อ่านข้อมูลจาก MPU6050
- **บรรทัด 632**: `ax_sum += ax;` - รวมค่า accelerometer X
- **บรรทัด 633**: `ay_sum += ay;` - รวมค่า accelerometer Y
- **บรรทัด 634**: `az_sum += az;` - รวมค่า accelerometer Z
- **บรรทัด 635**: `gx_sum += gx;` - รวมค่า gyroscope X
- **บรรทัด 636**: `gy_sum += gy;` - รวมค่า gyroscope Y
- **บรรทัด 637**: `gz_sum += gz;` - รวมค่า gyroscope Z
- **บรรทัด 638**: คอมเมนต์ว่าง
- **บรรทัด 639**: คอมเมนต์อธิบายการแสดงความคืบหน้า
- **บรรทัด 640**: `if (i % 300 == 0) {` - ตรวจสอบว่าถึงเวลาที่ต้องแสดงความคืบหน้าหรือไม่
- **บรรทัด 641**: `Serial.printf("   📊 Progress: %d/%d samples (%.1f%%)\n", i, samples, (float)i/samples*100);` - แสดงความคืบหน้า
- **บรรทัด 642**: `}` - ปิด if
- **บรรทัด 643**: `delay(10);` - หน่วงเวลา 10ms
- **บรรทัด 644**: `}` - ปิด for loop
- **บรรทัด 645**: คอมเมนต์ว่าง
- **บรรทัด 646**: คอมเมนต์อธิบายการคาลิเบรตแบบมืออาชีพ
- **บรรทัด 647**: `ax_offset = ax_sum / samples;` - คำนวณค่า offset ของ accelerometer X
- **บรรทัด 648**: `ay_offset = ay_sum / samples;` - คำนวณค่า offset ของ accelerometer Y
- **บรรทัด 649**: `az_offset = az_sum / samples - 16384;` - คำนวณค่า offset ของ accelerometer Z
- **บรรทัด 650**: คอมเมนต์อธิบาย Z-axis = 0 เมื่อวางราบ
- **บรรทัด 651**: คอมเมนต์ว่าง
- **บรรทัด 652**: คอมเมนต์อธิบายการคาลิเบรต gyroscope
- **บรรทัด 653**: `gx_offset = gx_sum / samples;` - คำนวณค่า offset ของ gyroscope X
- **บรรทัด 654**: `gy_offset = gy_sum / samples;` - คำนวณค่า offset ของ gyroscope Y
- **บรรทัด 655**: `gz_offset = gz_sum / samples;` - คำนวณค่า offset ของ gyroscope Z
- **บรรทัด 656**: คอมเมนต์ว่าง
- **บรรทัด 657**: `calibrated = true;` - ตั้งค่าการคาลิเบรตเสร็จสิ้น
- **บรรทัด 658**: `Serial.println("✅ PROFESSIONAL MPU6050 Calibration Complete!");` - แสดงข้อความการคาลิเบรตเสร็จสิ้น
- **บรรทัด 659**: `}` - ปิดฟังก์ชัน calibrateMPU6050
- **บรรทัด 660**: คอมเมนต์ว่าง
- **บรรทัด 661**: `void calculateElevationAngle() {` - ฟังก์ชันคำนวณมุมเงย
- **บรรทัด 662**: คอมเมนต์อธิบายการคำนวณมุมเงยแบบมืออาชีพ
- **บรรทัด 663**: คอมเมนต์ว่าง
- **บรรทัด 664**: คอมเมนต์อธิบายการควบคุม Sampling Rate
- **บรรทัด 665**: `unsigned long current_time = millis();` - เก็บเวลาปัจจุบัน
- **บรรทัด 666**: `if (current_time - last_sample_time < SAMPLE_INTERVAL) {` - ตรวจสอบว่าถึงเวลาสุ่มตัวอย่างหรือไม่
- **บรรทัด 667**: `return;` - ออกจากฟังก์ชันถ้ายังไม่ถึงเวลา
- **บรรทัด 668**: คอมเมนต์อธิบายการข้ามการสุ่มตัวอย่าง
- **บรรทัด 669**: `}` - ปิด if
- **บรรทัด 670**: `last_sample_time = current_time;` - อัปเดตเวลาสุ่มตัวอย่างครั้งล่าสุด
- **บรรทัด 671**: คอมเมนต์ว่าง
- **บรรทัด 672**: คอมเมนต์อธิบายการอ่านข้อมูลใหม่
- **บรรทัด 673**: `mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);` - อ่านข้อมูลจาก MPU6050
- **บรรทัด 674**: คอมเมนต์ว่าง
- **บรรทัด 675**: คอมเมนต์อธิบายการคาลิเบรตแบบมืออาชีพ
- **บรรทัด 676**: `if (calibrated) {` - ตรวจสอบว่าคาลิเบรตแล้วหรือไม่
- **บรรทัด 677**: `ax -= ax_offset;` - ลบค่า offset ของ accelerometer X
- **บรรทัด 678**: `ay -= ay_offset;` - ลบค่า offset ของ accelerometer Y
- **บรรทัด 679**: `az -= az_offset;` - ลบค่า offset ของ accelerometer Z
- **บรรทัด 680**: `gx -= gx_offset;` - ลบค่า offset ของ gyroscope X
- **บรรทัด 681**: `gy -= gy_offset;` - ลบค่า offset ของ gyroscope Y
- **บรรทัด 682**: `gz -= gz_offset;` - ลบค่า offset ของ gyroscope Z
- **บรรทัด 683**: `}` - ปิด if
- **บรรทัด 684**: คอมเมนต์ว่าง
- **บรรทัด 685**: คอมเมนต์อธิบายการแปลงค่า accelerometer
- **บรรทัด 686**: `float Ax = ax / 16384.0;` - แปลงค่า accelerometer X เป็นหน่วย g
- **บรรทัด 687**: คอมเมนต์อธิบาย MPU6050 ±2g range
- **บรรทัด 688**: `float Ay = ay / 16384.0;` - แปลงค่า accelerometer Y เป็นหน่วย g
- **บรรทัด 689**: `float Az = az / 16384.0;` - แปลงค่า accelerometer Z เป็นหน่วย g
- **บรรทัด 690**: คอมเมนต์ว่าง
- **บรรทัด 691**: คอมเมนต์อธิบายการแปลงค่า gyroscope
- **บรรทัด 692**: `float Gx = gx / GYRO_SENSITIVITY;` - แปลงค่า gyroscope X เป็นหน่วย °/s
- **บรรทัด 693**: คอมเมนต์อธิบาย MPU6050 ±250°/s range
- **บรรทัด 694**: `float Gy = gy / GYRO_SENSITIVITY;` - แปลงค่า gyroscope Y เป็นหน่วย °/s
- **บรรทัด 695**: `float Gz = gz / GYRO_SENSITIVITY;` - แปลงค่า gyroscope Z เป็นหน่วย °/s
- **บรรทัด 696**: คอมเมนต์ว่าง
- **บรรทัด 697**: คอมเมนต์อธิบายการคำนวณ Data Quality Score
- **บรรทัด 698**: `data_quality_score = calculateDataQuality(Ax, Ay, Az, Gx, Gy, Gz);` - คำนวณคะแนนคุณภาพข้อมูล
- **บรรทัด 699**: `high_quality_data = (data_quality_score > 80.0);` - ตรวจสอบว่าข้อมูลมีคุณภาพสูงหรือไม่
- **บรรทัด 700**: คอมเมนต์อธิบาย 80% threshold
- **บรรทัด 701**: คอมเมนต์ว่าง
- **บรรทัด 702**: คอมเมนต์อธิบายการคำนวณมุมเงยแบบมืออาชีพ
- **บรรทัด 703**: คอมเมนต์ว่าง
- **บรรทัด 704**: คอมเมนต์อธิบายการคำนวณมุมเงยจาก Accelerometer
- **บรรทัด 705**: `float total_magnitude = sqrt(Ax*Ax + Ay*Ay + Az*Az);` - คำนวณขนาดของเวกเตอร์ความเร่งรวม
- **บรรทัด 706**: คอมเมนต์ว่าง
- **บรรทัด 707**: `if (total_magnitude > 0.1 && high_quality_data) {` - ตรวจสอบว่าขนาดความเร่งและคุณภาพข้อมูลเพียงพอหรือไม่
- **บรรทัด 708**: คอมเมนต์อธิบายการตรวจสอบมุม 90°
- **บรรทัด 709**: `if (abs(Ax) > 0.95 && sqrt(Ay*Ay + Az*Az) < 0.2) {` - ตรวจสอบว่าใกล้มุม 90° หรือไม่
- **บรรทัด 710**: คอมเมนต์อธิบายการตรวจสอบโดยตรง
- **บรรทัด 711**: `if (Ax > 0) {` - ตรวจสอบว่าเงยขึ้นหรือไม่
- **บรรทัด 712**: `elevation_raw = 90.0;` - ตั้งมุมเงยเป็น 90°
- **บรรทัด 713**: คอมเมนต์อธิบายเงยขึ้น 90°
- **บรรทัด 714**: `} else {` - กรณีก้มลง
- **บรรทัด 715**: `elevation_raw = -90.0;` - ตั้งมุมเงยเป็น -90°
- **บรรทัด 716**: คอมเมนต์อธิบายก้มลง 90°
- **บรรทัด 717**: `}` - ปิด else
- **บรรทัด 718**: `} else {` - กรณีมุมอื่นๆ
- **บรรทัด 719**: คอมเมนต์อธิบายการใช้ atan2
- **บรรทัด 720**: `float yz_magnitude = sqrt(Ay*Ay + Az*Az);` - คำนวณขนาดของเวกเตอร์ในระนาบ YZ
- **บรรทัด 721**: `if (yz_magnitude > 0.1) {` - ตรวจสอบว่าขนาด YZ เพียงพอหรือไม่
- **บรรทัด 722**: `elevation_raw = atan2(Ax, yz_magnitude) * 180.0 / PI;` - คำนวณมุมเงยด้วย atan2
- **บรรทัด 723**: `} else {` - กรณีที่ขนาด YZ ไม่เพียงพอ
- **บรรทัด 724**: `elevation_raw = 0.0;` - ตั้งมุมเงยเป็น 0
- **บรรทัด 725**: คอมเมนต์อธิบายไม่มีข้อมูลที่เชื่อถือได้
- **บรรทัด 726**: `}` - ปิด else
- **บรรทัด 727**: `}` - ปิด else
- **บรรทัด 728**: `} else {` - กรณีที่ขนาดความเร่งหรือคุณภาพข้อมูลไม่เพียงพอ
- **บรรทัด 729**: `elevation_raw = 0.0;` - ตั้งมุมเงยเป็น 0
- **บรรทัด 730**: คอมเมนต์อธิบายไม่มีข้อมูลที่เชื่อถือได้
- **บรรทัด 731**: `}` - ปิด else
- **บรรทัด 732**: คอมเมนต์ว่าง
- **บรรทัด 733**: คอมเมนต์อธิบายการจำกัดค่ามุม
- **บรรทัด 734**: `if (elevation_raw > 90.0) elevation_raw = 90.0;` - จำกัดมุมเงยไม่เกิน 90°
- **บรรทัด 735**: `if (elevation_raw < -90.0) elevation_raw = -90.0;` - จำกัดมุมเงยไม่ต่ำกว่า -90°
- **บรรทัด 736**: คอมเมนต์ว่าง
- **บรรทัด 737**: คอมเมนต์อธิบายการใช้ Complementary Filter
- **บรรทัด 738**: คอมเมนต์อธิบาย Gyroscope rate สำหรับ pitch
- **บรรทัด 739**: `float gyro_rate = Gy;` - ใช้ค่า gyroscope Y เป็นอัตราการหมุน
- **บรรทัด 740**: คอมเมนต์อธิบาย °/s
- **บรรทัด 741**: คอมเมนต์ว่าง
- **บรรทัด 742**: คอมเมนต์อธิบายการใช้ Complementary Filter
- **บรรทัด 743**: `elevation_complementary = applyComplementaryFilter(elevation_raw, gyro_rate, DT);` - ใช้ Complementary Filter
- **บรรทัด 744**: คอมเมนต์อธิบายความแม่นยำ ±0.1°
- **บรรทัด 745**: คอมเมนต์ว่าง
- **บรรทัด 746**: คอมเมนต์อธิบายการจำกัดค่าผลลัพธ์
- **บรรทัด 747**: `if (elevation_complementary > 90.0) elevation_complementary = 90.0;` - จำกัดมุมเงยไม่เกิน 90°
- **บรรทัด 748**: `if (elevation_complementary < -90.0) elevation_complementary = -90.0;` - จำกัดมุมเงยไม่ต่ำกว่า -90°
- **บรรทัด 749**: `}` - ปิดฟังก์ชัน calculateElevationAngle

### 15. ฟังก์ชัน BLE (บรรทัด 894-981)
```cpp
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

    // ส่งข้อมูลมุมเงยแบบมืออาชีพ (Elevation Angle) ไปมือถือ
    String msg = "elevation:" + String((int)elevation_complementary); // จำนวนเต็ม (ไม่มีทศนิยม)
    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
    delay(50);

    // ส่งระยะทางจาก PTFS sensor
    msg = "distance:" + String(distance, 1);
    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
    delay(50);

    // ส่ง Data Quality Score
    msg = "quality:" + String(data_quality_score, 1);
    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
    delay(50);

    // ส่ง mode
    msg = "mode:" + String(currentMode);
    pCharacteristic->setValue(msg.c_str());
    pCharacteristic->notify();
    delay(50);

    // ส่งสัญญาณจบ
    pCharacteristic->setValue("END");
    pCharacteristic->notify();
}

void handleBLEReconnection() {
    // ปรับปรุงการจัดการ BLE
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
```

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 894**: คอมเมนต์หัวข้อสำหรับ BLE Functions
- **บรรทัด 895**: `void setupBLE() {` - ฟังก์ชันตั้งค่า BLE
- **บรรทัด 896**: `Serial.println("🔵 Starting BLE system...");` - แสดงข้อความเริ่มต้น BLE
- **บรรทัด 897**: คอมเมนต์ว่าง
- **บรรทัด 898**: `BLEDevice::init(DEVICE_NAME);` - เริ่มต้น BLE device ด้วยชื่อที่กำหนด
- **บรรทัด 899**: `BLEDevice::setMTU(247);` - ตั้งค่า MTU (Maximum Transmission Unit) เป็น 247 bytes
- **บรรทัด 900**: `esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);` - ปลดปล่อยหน่วยความจำของ Classic Bluetooth
- **บรรทัด 901**: คอมเมนต์ว่าง
- **บรรทัด 902**: `pServer = BLEDevice::createServer();` - สร้าง BLE Server
- **บรรทัด 903**: `pServer->setCallbacks(new MyServerCallbacks());` - ตั้งค่า callbacks สำหรับ BLE Server
- **บรรทัด 904**: คอมเมนต์ว่าง
- **บรรทัด 905**: `BLEService *pService = pServer->createService(SERVICE_UUID);` - สร้าง BLE Service ด้วย UUID ที่กำหนด
- **บรรทัด 906**: คอมเมนต์ว่าง
- **บรรทัด 907**: `pCharacteristic = pService->createCharacteristic(` - สร้าง BLE Characteristic
- **บรรทัด 908**: `CHARACTERISTIC_UUID,` - ใช้ UUID ที่กำหนด
- **บรรทัด 909**: `BLECharacteristic::PROPERTY_READ   |` - ตั้งค่าให้อ่านได้
- **บรรทัด 910**: `BLECharacteristic::PROPERTY_WRITE  |` - ตั้งค่าให้เขียนได้
- **บรรทัด 911**: `BLECharacteristic::PROPERTY_NOTIFY |` - ตั้งค่าให้แจ้งเตือนได้
- **บรรทัด 912**: `BLECharacteristic::PROPERTY_INDICATE` - ตั้งค่าให้ส่งสัญญาณได้
- **บรรทัด 913**: `);` - ปิดการสร้าง Characteristic
- **บรรทัด 914**: คอมเมนต์ว่าง
- **บรรทัด 915**: `pCharacteristic->addDescriptor(new BLE2902());` - เพิ่ม descriptor สำหรับ Notify/Indicate
- **บรรทัด 916**: `pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());` - ตั้งค่า callbacks สำหรับ Characteristic
- **บรรทัด 917**: คอมเมนต์ว่าง
- **บรรทัด 918**: `pService->start();` - เริ่มต้น BLE Service
- **บรรทัด 919**: คอมเมนต์ว่าง
- **บรรทัด 920**: `BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();` - สร้าง BLE Advertising
- **บรรทัด 921**: `pAdvertising->addServiceUUID(SERVICE_UUID);` - เพิ่ม Service UUID ในการโฆษณา
- **บรรทัด 922**: `pAdvertising->setScanResponse(true);` - เปิดใช้งาน scan response
- **บรรทัด 923**: `pAdvertising->setMinPreferred(0x06);` - ตั้งค่าความถี่การโฆษณาต่ำสุด
- **บรรทัด 924**: `pAdvertising->setMinPreferred(0x12);` - ตั้งค่าความถี่การโฆษณาสูงสุด
- **บรรทัด 925**: `BLEDevice::startAdvertising();` - เริ่มต้นการโฆษณา BLE
- **บรรทัด 926**: คอมเมนต์ว่าง
- **บรรทัด 927**: `Serial.println("✅ BLE ready, waiting for connection...");` - แสดงข้อความ BLE พร้อมใช้งาน
- **บรรทัด 928**: `}` - ปิดฟังก์ชัน setupBLE
- **บรรทัด 929**: คอมเมนต์ว่าง
- **บรรทัด 930**: `void sendBLEData() {` - ฟังก์ชันส่งข้อมูลผ่าน BLE
- **บรรทัด 931**: `if (!deviceConnected || !pCharacteristic) return;` - ตรวจสอบว่ามีการเชื่อมต่อและ Characteristic หรือไม่
- **บรรทัด 932**: คอมเมนต์ว่าง
- **บรรทัด 933**: คอมเมนต์อธิบายการส่งข้อมูลมุมเงย
- **บรรทัด 934**: `String msg = "elevation:" + String((int)elevation_complementary);` - สร้างข้อความมุมเงย
- **บรรทัด 935**: คอมเมนต์อธิบายจำนวนเต็ม
- **บรรทัด 936**: `pCharacteristic->setValue(msg.c_str());` - ตั้งค่าข้อความใน Characteristic
- **บรรทัด 937**: `pCharacteristic->notify();` - ส่งการแจ้งเตือน
- **บรรทัด 938**: `delay(50);` - หน่วงเวลา 50ms
- **บรรทัด 939**: คอมเมนต์ว่าง
- **บรรทัด 940**: คอมเมนต์อธิบายการส่งระยะทาง
- **บรรทัด 941**: `msg = "distance:" + String(distance, 1);` - สร้างข้อความระยะทาง
- **บรรทัด 942**: `pCharacteristic->setValue(msg.c_str());` - ตั้งค่าข้อความใน Characteristic
- **บรรทัด 943**: `pCharacteristic->notify();` - ส่งการแจ้งเตือน
- **บรรทัด 944**: `delay(50);` - หน่วงเวลา 50ms
- **บรรทัด 945**: คอมเมนต์ว่าง
- **บรรทัด 946**: คอมเมนต์อธิบายการส่ง Data Quality Score
- **บรรทัด 947**: `msg = "quality:" + String(data_quality_score, 1);` - สร้างข้อความคุณภาพข้อมูล
- **บรรทัด 948**: `pCharacteristic->setValue(msg.c_str());` - ตั้งค่าข้อความใน Characteristic
- **บรรทัด 949**: `pCharacteristic->notify();` - ส่งการแจ้งเตือน
- **บรรทัด 950**: `delay(50);` - หน่วงเวลา 50ms
- **บรรทัด 951**: คอมเมนต์ว่าง
- **บรรทัด 952**: คอมเมนต์อธิบายการส่ง mode
- **บรรทัด 953**: `msg = "mode:" + String(currentMode);` - สร้างข้อความโหมด
- **บรรทัด 954**: `pCharacteristic->setValue(msg.c_str());` - ตั้งค่าข้อความใน Characteristic
- **บรรทัด 955**: `pCharacteristic->notify();` - ส่งการแจ้งเตือน
- **บรรทัด 956**: `delay(50);` - หน่วงเวลา 50ms
- **บรรทัด 957**: คอมเมนต์ว่าง
- **บรรทัด 958**: คอมเมนต์อธิบายการส่งสัญญาณจบ
- **บรรทัด 959**: `pCharacteristic->setValue("END");` - ตั้งค่าข้อความจบ
- **บรรทัด 960**: `pCharacteristic->notify();` - ส่งการแจ้งเตือน
- **บรรทัด 961**: `}` - ปิดฟังก์ชัน sendBLEData
- **บรรทัด 962**: คอมเมนต์ว่าง
- **บรรทัด 963**: `void handleBLEReconnection() {` - ฟังก์ชันจัดการการเชื่อมต่อ BLE ใหม่
- **บรรทัด 964**: คอมเมนต์อธิบายการปรับปรุงการจัดการ BLE
- **บรรทัด 965**: `static unsigned long lastReconnectAttempt = 0;` - ตัวแปรเก็บเวลาพยายามเชื่อมต่อครั้งล่าสุด
- **บรรทัด 966**: `const unsigned long reconnectInterval = 2000;` - ค่าคงที่สำหรับช่วงเวลาระหว่างการพยายามเชื่อมต่อ
- **บรรทัด 967**: คอมเมนต์อธิบาย 2 วินาที
- **บรรทัด 968**: คอมเมนต์ว่าง
- **บรรทัด 969**: `if (!deviceConnected && oldDeviceConnected) {` - ตรวจสอบว่าอุปกรณ์ตัดการเชื่อมต่อหรือไม่
- **บรรทัด 970**: คอมเมนต์อธิบายการจำกัดความถี่
- **บรรทัด 971**: `if (millis() - lastReconnectAttempt >= reconnectInterval) {` - ตรวจสอบว่าถึงเวลาพยายามเชื่อมต่อใหม่หรือไม่
- **บรรทัด 972**: `delay(500);` - หน่วงเวลา 500ms
- **บรรทัด 973**: `pServer->startAdvertising();` - เริ่มต้นการโฆษณา BLE ใหม่
- **บรรทัด 974**: `Serial.println("🔄 Start advertising again...");` - แสดงข้อความเริ่มต้นการโฆษณาใหม่
- **บรรทัด 975**: `lastReconnectAttempt = millis();` - อัปเดตเวลาพยายามเชื่อมต่อครั้งล่าสุด
- **บรรทัด 976**: `}` - ปิด if
- **บรรทัด 977**: `oldDeviceConnected = deviceConnected;` - อัปเดตสถานะการเชื่อมต่อก่อนหน้า
- **บรรทัด 978**: `}` - ปิด if
- **บรรทัด 979**: `if (deviceConnected && !oldDeviceConnected) {` - ตรวจสอบว่าอุปกรณ์เชื่อมต่อใหม่หรือไม่
- **บรรทัด 980**: `oldDeviceConnected = deviceConnected;` - อัปเดตสถานะการเชื่อมต่อก่อนหน้า
- **บรรทัด 981**: `Serial.println("🔵 BLE Reconnected Successfully!");` - แสดงข้อความเชื่อมต่อสำเร็จ
- **บรรทัด 982**: `}` - ปิด if
- **บรรทัด 983**: `}` - ปิดฟังก์ชัน handleBLEReconnection

### 16. ฟังก์ชัน Setup (บรรทัด 983-1094)
```cpp
// ================ Setup ================
void setup() {
    Serial.begin(115200);
    Serial.println("🎯 ESP32 + MPU6050 + PTFS Distance Sensor + BLE - PROFESSIONAL EDITION");
    Serial.println("=======================================================================");
    Serial.println("📐 PROFESSIONAL ELEVATION/DEPRESSION ANGLE measurement with ±0.1° precision");
    
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

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 983**: คอมเมนต์หัวข้อสำหรับ Setup
- **บรรทัด 984**: `void setup() {` - ฟังก์ชัน setup ที่เรียกครั้งเดียวเมื่อเริ่มต้น
- **บรรทัด 985**: `Serial.begin(115200);` - เริ่มต้น Serial communication ที่ 115200 bps
- **บรรทัด 986**: `Serial.println("🎯 ESP32 + MPU6050 + PTFS Distance Sensor + BLE - PROFESSIONAL EDITION");` - แสดงข้อความหัวข้อโปรเจค
- **บรรทัด 987**: `Serial.println("=======================================================================");` - แสดงเส้นแบ่ง
- **บรรทัด 988**: `Serial.println("📐 PROFESSIONAL ELEVATION/DEPRESSION ANGLE measurement with ±0.1° precision");` - แสดงข้อความอธิบายความแม่นยำ
- **บรรทัด 989**: คอมเมนต์ว่าง
- **บรรทัด 990**: คอมเมนต์อธิบายการเริ่มต้น I2C
- **บรรทัด 991**: `Wire.begin(SDA_PIN, SCL_PIN);` - เริ่มต้น I2C communication
- **บรรทัด 992**: `Wire.setClock(400000);` - ตั้งค่าความถี่ I2C เป็น 400kHz
- **บรรทัด 993**: คอมเมนต์ว่าง
- **บรรทัด 994**: คอมเมนต์อธิบายการตั้งค่า PTFS Distance Sensor
- **บรรทัด 995**: `Serial.println("🔧 Initialize PTFS Distance Sensor (3.3V system)...");` - แสดงข้อความเริ่มต้น PTFS
- **บรรทัด 996**: `Serial.println("   ⚡ PTFS VIN → 3.3V, PWR_EN → GPIO2 (HIGH)");` - แสดงข้อความการเชื่อมต่อไฟ
- **บรรทัด 997**: `Serial.println("   🔌 TX → GPIO16 (RX2), RX → GPIO17 (TX2)");` - แสดงข้อความการเชื่อมต่อ UART
- **บรรทัด 998**: `Serial.println("   📡 UART: 115200 bps, TTL 3.3V");` - แสดงข้อความการตั้งค่า UART
- **บรรทัด 999**: คอมเมนต์ว่าง
- **บรรทัด 1000**: `pinMode(PTFS_PWR_PIN, OUTPUT);` - ตั้งค่า GPIO2 เป็น output
- **บรรทัด 1001**: `pinMode(PTFS_RST_PIN, OUTPUT);` - ตั้งค่า GPIO4 เป็น output
- **บรรทัด 1002**: คอมเมนต์ว่าง
- **บรรทัด 1003**: `digitalWrite(PTFS_PWR_PIN, HIGH);` - เปิด PTFS ด้วย 3.3V
- **บรรทัด 1004**: คอมเมนต์อธิบายการเปิด PTFS
- **บรรทัด 1005**: `digitalWrite(PTFS_RST_PIN, LOW);` - ตั้งค่า reset เป็น LOW
- **บรรทัด 1006**: `delay(100);` - หน่วงเวลา 100ms
- **บรรทัด 1007**: `digitalWrite(PTFS_RST_PIN, HIGH);` - ตั้งค่า reset เป็น HIGH
- **บรรทัด 1008**: `delay(500);` - หน่วงเวลา 500ms
- **บรรทัด 1009**: คอมเมนต์ว่าง
- **บรรทัด 1010**: คอมเมนต์อธิบายการเริ่มต้น UART
- **บรรทัด 1011**: `Serial2.begin(115200, SERIAL_8N1, PTFS_RX_PIN, PTFS_TX_PIN);` - เริ่มต้น UART2 ที่ 115200 bps
- **บรรทัด 1012**: คอมเมนต์ว่าง
- **บรรทัด 1013**: คอมเมนต์อธิบายการตั้งค่า GPIO signal strength
- **บรรทัด 1014**: `gpio_set_drive_capability(GPIO_NUM_17, GPIO_DRIVE_CAP_2);` - ตั้งค่า drive capability ของ GPIO17
- **บรรทัด 1015**: คอมเมนต์อธิบาย 3.3V TTL
- **บรรทัด 1016**: `gpio_set_pull_mode(GPIO_NUM_17, GPIO_PULLUP_ONLY);` - ตั้งค่า pull-up ของ GPIO17
- **บรรทัด 1017**: `gpio_set_drive_capability(GPIO_NUM_16, GPIO_DRIVE_CAP_2);` - ตั้งค่า drive capability ของ GPIO16
- **บรรทัด 1018**: `gpio_set_pull_mode(GPIO_NUM_16, GPIO_PULLUP_ONLY);` - ตั้งค่า pull-up ของ GPIO16
- **บรรทัด 1019**: คอมเมนต์ว่าง
- **บรรทัด 1020**: `Serial.println("✅ PTFS Distance Sensor ready (3.3V system)");` - แสดงข้อความ PTFS พร้อมใช้งาน
- **บรรทัด 1021**: คอมเมนต์ว่าง
- **บรรทัด 1022**: `delay(1000);` - หน่วงเวลา 1 วินาที
- **บรรทัด 1023**: คอมเมนต์ว่าง
- **บรรทัด 1024**: คอมเมนต์อธิบายการเริ่มต้น MPU6050
- **บรรทัด 1025**: `Serial.println("🔧 Initialize MPU6050...");` - แสดงข้อความเริ่มต้น MPU6050
- **บรรทัด 1026**: `mpu.initialize();` - เริ่มต้น MPU6050
- **บรรทัด 1027**: `mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);` - ตั้งค่า accelerometer range เป็น ±2g
- **บรรทัด 1028**: `mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);` - ตั้งค่า gyroscope range เป็น ±250°/s
- **บรรทัด 1029**: `mpu.setDLPFMode(MPU6050_DLPF_BW_5);` - ตั้งค่า Digital Low Pass Filter
- **บรรทัด 1030**: คอมเมนต์ว่าง
- **บรรทัด 1031**: `if (mpu.testConnection()) {` - ตรวจสอบการเชื่อมต่อ MPU6050
- **บรรทัด 1032**: `Serial.println("✅ MPU6050 Connected Successfully!");` - แสดงข้อความเชื่อมต่อสำเร็จ
- **บรรทัด 1033**: คอมเมนต์ว่าง
- **บรรทัด 1034**: คอมเมนต์อธิบายการเริ่มต้นการคำนวณมุมเงย
- **บรรทัด 1035**: `initializeAdvancedPitch();` - เรียกฟังก์ชันเริ่มต้นการคำนวณมุมเงย
- **บรรทัด 1036**: คอมเมนต์ว่าง
- **บรรทัด 1037**: คอมเมนต์อธิบายการคาลิเบรต MPU6050
- **บรรทัด 1038**: `calibrateMPU6050();` - เรียกฟังก์ชันคาลิเบรต MPU6050
- **บรรทัด 1039**: คอมเมนต์ว่าง
- **บรรทัด 1040**: คอมเมนต์อธิบายการตั้งค่า BLE
- **บรรทัด 1041**: `setupBLE();` - เรียกฟังก์ชันตั้งค่า BLE
- **บรรทัด 1042**: คอมเมนต์ว่าง
- **บรรทัด 1043**: คอมเมนต์อธิบายการเริ่มต้นการวัดระยะทาง PTFS
- **บรรทัด 1044**: `Serial.println("📏 Starting PTFS distance measurement...");` - แสดงข้อความเริ่มต้นการวัดระยะทาง
- **บรรทัด 1045**: `Serial.println("   🎯 Using official PTFS protocol (0xFA/0xFB)");` - แสดงข้อความอธิบายโปรโตคอล
- **บรรทัด 1046**: `Serial.println("   📡 Baud rate: 115200 bps");` - แสดงข้อความอธิบายความเร็ว
- **บรรทัด 1047**: คอมเมนต์ว่าง
- **บรรทัด 1048**: คอมเมนต์อธิบายการเริ่มส่งคำสั่ง PTFS Protocol
- **บรรทัด 1049**: `ptfsMeasurementActive = true;` - เปิดใช้งานการวัดระยะทาง PTFS
- **บรรทัด 1050**: `startPTFSMeasurement();` - เริ่มต้นการวัดระยะทาง PTFS
- **บรรทัด 1051**: `lastPTFSCommand = millis();` - อัปเดตเวลาส่งคำสั่ง PTFS ครั้งล่าสุด
- **บรรทัด 1052**: `lastPTFSAnalysis = millis();` - อัปเดตเวลาวิเคราะห์ข้อมูล PTFS ครั้งล่าสุด
- **บรรทัด 1053**: `lastPTFSDisplayTime = millis();` - อัปเดตเวลแสดงผลข้อมูล PTFS ครั้งล่าสุด
- **บรรทัด 1054**: คอมเมนต์ว่าง
- **บรรทัด 1055**: `Serial.println("=======================================================================");` - แสดงเส้นแบ่ง
- **บรรทัด 1056**: `Serial.println("🎯 PROFESSIONAL SYSTEM READY! Measuring with ±0.1° precision...");` - แสดงข้อความระบบพร้อมใช้งาน
- **บรรทัด 1057**: `Serial.println("📐 ELEVATION: มุมเงย/มุมก้ม (Elevation/Depression) in degrees via BLE");` - แสดงข้อความอธิบายมุมเงย
- **บรรทัด 1058**: `Serial.println("📏 PTFS: 3.3V power, UART 115200 bps, Official protocol");` - แสดงข้อความอธิบาย PTFS
- **บรรทัด 1059**: `Serial.println("📊 QUALITY: Real-time data quality assessment (0-100%)");` - แสดงข้อความอธิบายคุณภาพข้อมูล
- **บรรทัด 1060**: `Serial.println("🎯 FOCUS: Professional-grade Elevation/Depression Angle measurement");` - แสดงข้อความอธิบายจุดเน้น
- **บรรทัด 1061**: `Serial.println("🔒 STABLE: Roll Independent - มุมเงยไม่เปลี่ยนเมื่อเอียงซ้ายขวา");` - แสดงข้อความอธิบายความเสถียร
- **บรรทัด 1062**: `Serial.println("⚡ SAMPLING: 100 Hz controlled rate for maximum accuracy");` - แสดงข้อความอธิบายอัตราการสุ่มตัวอย่าง
- **บรรทัด 1063**: คอมเมนต์ว่าง
- **บรรทัด 1064**: `} else {` - กรณีที่เชื่อมต่อ MPU6050 ไม่สำเร็จ
- **บรรทัด 1065**: `Serial.println("❌ MPU6050 Connection Failed!");` - แสดงข้อความเชื่อมต่อล้มเหลว
- **บรรทัด 1066**: `Serial.println("Check wiring:");` - แสดงข้อความตรวจสอบการเชื่อมต่อ
- **บรรทัด 1067**: `Serial.println("  ESP32 3.3V → MPU6050 VCC");` - แสดงข้อความการเชื่อมต่อไฟ
- **บรรทัด 1068**: `Serial.println("  ESP32 GND  → MPU6050 GND");` - แสดงข้อความการเชื่อมต่อกราวด์
- **บรรทัด 1069**: `Serial.println("  ESP32 G21  → MPU6050 SDA");` - แสดงข้อความการเชื่อมต่อ SDA
- **บรรทัด 1070**: `Serial.println("  ESP32 G22  → MPU6050 SCL");` - แสดงข้อความการเชื่อมต่อ SCL
- **บรรทัด 1071**: `Serial.println("  PTFS VIN   → 3.3V");` - แสดงข้อความการเชื่อมต่อไฟ PTFS
- **บรรทัด 1072**: `Serial.println("  PTFS GND   → GND");` - แสดงข้อความการเชื่อมต่อกราวด์ PTFS
- **บรรทัด 1073**: `Serial.println("  PTFS TX    → GPIO16 (RX2)");` - แสดงข้อความการเชื่อมต่อ TX PTFS
- **บรรทัด 1074**: `Serial.println("  PTFS RX    → GPIO17 (TX2) - No R-divider needed");` - แสดงข้อความการเชื่อมต่อ RX PTFS
- **บรรทัด 1075**: `while(1);` - วนลูปไม่สิ้นสุด
- **บรรทัด 1076**: `}` - ปิด else
- **บรรทัด 1077**: `}` - ปิดฟังก์ชัน setup

### 17. ฟังก์ชัน Main Loop (บรรทัด 1096-1229)
```cpp
// ================ Main Loop ================
void loop() {
    // ปรับปรุงการจัดการ Watchdog Timer
    static unsigned long lastWatchdogFeed = 0;
    
    // ให้อาหาร watchdog บ่อยขึ้น
    if (millis() - lastWatchdogFeed >= 1000) { // ทุก 1 วินาที
        yield();
        lastWatchdogFeed = millis();
    }
    
    // เพิ่มการตรวจสอบสถานะระบบ
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

**คำอธิบายแต่ละบรรทัด:**
- **บรรทัด 1096**: คอมเมนต์หัวข้อสำหรับ Main Loop
- **บรรทัด 1097**: `void loop() {` - ฟังก์ชัน loop ที่เรียกซ้ำๆ ตลอดเวลา
- **บรรทัด 1098**: คอมเมนต์อธิบายการปรับปรุงการจัดการ Watchdog Timer
- **บรรทัด 1099**: `static unsigned long lastWatchdogFeed = 0;` - ตัวแปรเก็บเวลาที่ให้อาหาร watchdog ครั้งล่าสุด
- **บรรทัด 1100**: คอมเมนต์ว่าง
- **บรรทัด 1101**: คอมเมนต์อธิบายการให้อาหาร watchdog บ่อยขึ้น
- **บรรทัด 1102**: `if (millis() - lastWatchdogFeed >= 1000) {` - ตรวจสอบว่าถึงเวลาที่ต้องให้อาหาร watchdog หรือไม่
- **บรรทัด 1103**: คอมเมนต์อธิบายทุก 1 วินาที
- **บรรทัด 1104**: `yield();` - ให้อาหาร watchdog
- **บรรทัด 1105**: `lastWatchdogFeed = millis();` - อัปเดตเวลาที่ให้อาหาร watchdog ครั้งล่าสุด
- **บรรทัด 1106**: `}` - ปิด if
- **บรรทัด 1107**: คอมเมนต์ว่าง
- **บรรทัด 1108**: คอมเมนต์อธิบายการตรวจสอบสถานะระบบ
- **บรรทัด 1109**: `performHealthCheck();` - เรียกฟังก์ชันตรวจสอบสถานะระบบ
- **บรรทัด 1110**: คอมเมนต์ว่าง
- **บรรทัด 1111**: คอมเมนต์อธิบายการประมวลผลข้อมูล PTFS
- **บรรทัด 1112**: `processPTFSStream();` - เรียกฟังก์ชันประมวลผลข้อมูล PTFS
- **บรรทัด 1113**: คอมเมนต์ว่าง
- **บรรทัด 1114**: คอมเมนต์อธิบายการส่งคำสั่ง PTFS แบบต่อเนื่อง
- **บรรทัด 1115**: `if (millis() - lastPTFSCommand >= ptfsCommandInterval) {` - ตรวจสอบว่าถึงเวลาที่ต้องส่งคำสั่ง PTFS หรือไม่
- **บรรทัด 1116**: `if (ptfsMeasurementActive) {` - ตรวจสอบว่า PTFS เปิดใช้งานหรือไม่
- **บรรทัด 1117**: `startPTFSMeasurement();` - เริ่มต้นการวัดระยะทาง PTFS
- **บรรทัด 1118**: `}` - ปิด if
- **บรรทัด 1119**: `lastPTFSCommand = millis();` - อัปเดตเวลาส่งคำสั่ง PTFS ครั้งล่าสุด
- **บรรทัด 1120**: `}` - ปิด if
- **บรรทัด 1121**: คอมเมนต์ว่าง
- **บรรทัด 1122**: คอมเมนต์อธิบายการรีเซ็ตเซ็นเซอร์ทุก 5 นาที
- **บรรทัด 1123**: `static unsigned long lastHardwareReset = 0;` - ตัวแปรเก็บเวลาที่รีเซ็ตเซ็นเซอร์ครั้งล่าสุด
- **บรรทัด 1124**: `if (millis() - lastHardwareReset >= 300000) {` - ตรวจสอบว่าถึงเวลาที่ต้องรีเซ็ตเซ็นเซอร์หรือไม่
- **บรรทัด 1125**: คอมเมนต์อธิบาย 5 นาที
- **บรรทัด 1126**: `digitalWrite(PTFS_RST_PIN, LOW);` - ตั้งค่า reset เป็น LOW
- **บรรทัด 1127**: `delay(100);` - หน่วงเวลา 100ms
- **บรรทัด 1128**: `digitalWrite(PTFS_RST_PIN, HIGH);` - ตั้งค่า reset เป็น HIGH
- **บรรทัด 1129**: `delay(500);` - หน่วงเวลา 500ms
- **บรรทัด 1130**: `ptfsMeasurementActive = true;` - เปิดใช้งานการวัดระยะทาง PTFS
- **บรรทัด 1131**: คอมเมนต์อธิบายการเปิดใช้งานใหม่
- **บรรทัด 1132**: `lastHardwareReset = millis();` - อัปเดตเวลาที่รีเซ็ตเซ็นเซอร์ครั้งล่าสุด
- **บรรทัด 1133**: `}` - ปิด if
- **บรรทัด 1134**: คอมเมนต์ว่าง
- **บรรทัด 1135**: คอมเมนต์อธิบายการอ่านข้อมูล MPU6050
- **บรรทัด 1136**: `if (mpu6050Healthy) {` - ตรวจสอบว่า MPU6050 ทำงานปกติหรือไม่
- **บรรทัด 1137**: `mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);` - อ่านข้อมูลจาก MPU6050
- **บรรทัด 1138**: `} else {` - กรณีที่ MPU6050 ไม่ทำงานปกติ
- **บรรทัด 1139**: `Serial.println("⚠️ Using cached MPU6050 data");` - แสดงข้อความใช้ข้อมูลที่เก็บไว้
- **บรรทัด 1140**: `}` - ปิด else
- **บรรทัด 1141**: คอมเมนต์ว่าง
- **บรรทัด 1142**: คอมเมนต์อธิบายการคำนวณมุมเงย
- **บรรทัด 1143**: `calculateElevationAngle();` - เรียกฟังก์ชันคำนวณมุมเงย
- **บรรทัด 1144**: คอมเมนต์ว่าง
- **บรรทัด 1145**: คอมเมนต์อธิบายการแสดงค่าปัจจุบัน
- **บรรทัด 1146**: `Serial.printf("📐 ELEVATION: %+8d°  |  DIST: %6.1fm  |  QUALITY: %5.1f%%",` - แสดงข้อความมุมเงย ระยะทาง และคุณภาพ
- **บรรทัด 1147**: `(int)elevation_complementary, distance, data_quality_score);` - แสดงค่ามุมเงย ระยะทาง และคุณภาพ
- **บรรทัด 1148**: คอมเมนต์ว่าง
- **บรรทัด 1149**: คอมเมนต์อธิบายการแสดงข้อมูลดิบ
- **บรรทัด 1150**: `static unsigned long lastRawDisplay = 0;` - ตัวแปรเก็บเวลาที่แสดงข้อมูลดิบครั้งล่าสุด
- **บรรทัด 1151**: `if (millis() - lastRawDisplay >= 3000) {` - ตรวจสอบว่าถึงเวลาที่ต้องแสดงข้อมูลดิบหรือไม่
- **บรรทัด 1152**: `Serial.printf("   📊 Raw: Ax=%+6.3f, Ay=%+6.3f, Az=%+6.3f (g)",` - แสดงข้อความข้อมูลดิบ accelerometer
- **บรรทัด 1153**: `ax/16384.0, ay/16384.0, az/16384.0);` - แสดงค่าข้อมูลดิบ accelerometer
- **บรรทัด 1154**: คอมเมนต์ว่าง
- **บรรทัด 1155**: คอมเมนต์อธิบายการทดสอบการทำงาน
- **บรรทัด 1156**: `if (elevation_complementary > 1.0) {` - ตรวจสอบว่ามุมเงยมากกว่า 1° หรือไม่
- **บรรทัด 1157**: `Serial.printf(" → เงยขึ้น");` - แสดงข้อความเงยขึ้น
- **บรรทัด 1158**: `} else if (elevation_complementary < -1.0) {` - ตรวจสอบว่ามุมเงยน้อยกว่า -1° หรือไม่
- **บรรทัด 1159**: `Serial.printf(" → ก้มลง");` - แสดงข้อความก้มลง
- **บรรทัด 1160**: `} else {` - กรณีอื่นๆ
- **บรรทัด 1161**: `Serial.printf(" → ราบ");` - แสดงข้อความราบ
- **บรรทัด 1162**: `}` - ปิด else
- **บรรทัด 1163**: คอมเมนต์ว่าง
- **บรรทัด 1164**: `lastRawDisplay = millis();` - อัปเดตเวลาที่แสดงข้อมูลดิบครั้งล่าสุด
- **บรรทัด 1165**: `}` - ปิด if
- **บรรทัด 1166**: คอมเมนต์ว่าง
- **บรรทัด 1167**: คอมเมนต์อธิบายการแสดงสถานะการเชื่อมต่อ
- **บรรทัด 1168**: `if (deviceConnected) {` - ตรวจสอบว่ามีการเชื่อมต่อ BLE หรือไม่
- **บรรทัด 1169**: `Serial.print("  [🔵 BLE Connected]");` - แสดงข้อความเชื่อมต่อ BLE
- **บรรทัด 1170**: `} else {` - กรณีที่ไม่มี BLE
- **บรรทัด 1171**: `Serial.print("  [⚪ BLE Disconnected]");` - แสดงข้อความไม่เชื่อมต่อ BLE
- **บรรทัด 1172**: `}` - ปิด else
- **บรรทัด 1173**: คอมเมนต์ว่าง
- **บรรทัด 1174**: คอมเมนต์อธิบายการแสดงสถานะเซ็นเซอร์
- **บรรทัด 1175**: `if (mpu6050Healthy && ptfsHealthy) {` - ตรวจสอบว่าเซ็นเซอร์ทำงานปกติหรือไม่
- **บรรทัด 1176**: `Serial.print("  [✅ Sensors OK]");` - แสดงข้อความเซ็นเซอร์ปกติ
- **บรรทัด 1177**: `} else {` - กรณีที่เซ็นเซอร์ไม่ปกติ
- **บรรทัด 1178**: `Serial.print("  [⚠️ Sensor Issues]");` - แสดงข้อความเซ็นเซอร์มีปัญหา
- **บรรทัด 1179**: `}` - ปิด else
- **บรรทัด 1180**: คอมเมนต์ว่าง
- **บรรทัด 1181**: `Serial.println();` - ขึ้นบรรทัดใหม่
- **บรรทัด 1182**: คอมเมนต์ว่าง
- **บรรทัด 1183**: คอมเมนต์อธิบายการส่งข้อมูล BLE
- **บรรทัด 1184**: `unsigned long currentMillis = millis();` - เก็บเวลาปัจจุบัน
- **บรรทัด 1185**: `if (currentMillis - lastSendTime >= sendInterval) {` - ตรวจสอบว่าถึงเวลาที่ต้องส่งข้อมูล BLE หรือไม่
- **บรรทัด 1186**: `lastSendTime = currentMillis;` - อัปเดตเวลาส่งข้อมูล BLE ครั้งล่าสุด
- **บรรทัด 1187**: `if (deviceConnected) {` - ตรวจสอบว่ามีการเชื่อมต่อ BLE หรือไม่
- **บรรทัด 1188**: `sendBLEData();` - เรียกฟังก์ชันส่งข้อมูล BLE
- **บรรทัด 1189**: `}` - ปิด if
- **บรรทัด 1190**: `}` - ปิด if
- **บรรทัด 1191**: คอมเมนต์ว่าง
- **บรรทัด 1192**: คอมเมนต์อธิบายการจัดการการเชื่อมต่อ BLE ใหม่
- **บรรทัด 1193**: `handleBLEReconnection();` - เรียกฟังก์ชันจัดการการเชื่อมต่อ BLE ใหม่
- **บรรทัด 1194**: คอมเมนต์ว่าง
- **บรรทัด 1195**: คอมเมนต์อธิบายการป้องกัน watchdog timer
- **บรรทัด 1196**: `yield();` - ให้อาหาร watchdog
- **บรรทัด 1197**: `delay(10);` - หน่วงเวลา 10ms
- **บรรทัด 1198**: `}` - ปิดฟังก์ชัน loop

## สรุปคุณสมบัติหลัก

### 1. ระบบวัดมุมเงยแบบมืออาชีพ
- **ความแม่นยำ**: ±0.1° ด้วย Complementary Filter
- **การคาลิเบรต**: 3,000 samples สำหรับความแม่นยำสูงสุด
- **อัตราการสุ่มตัวอย่าง**: 100 Hz (10ms interval)
- **การประเมินคุณภาพ**: 0-100% real-time

### 2. ระบบวัดระยะทาง PTFS
- **โปรโตคอล**: Official PTFS Protocol (0xFA/0xFB)
- **ความเร็ว**: 115200 bps UART
- **การกรอง**: Moving Average Filter 30 samples
- **ช่วงการวัด**: 3m-1000m

### 3. ระบบ BLE
- **การเชื่อมต่อ**: Auto-reconnection
- **ข้อมูลที่ส่ง**: elevation, distance, quality, mode
- **การจัดการ**: Health monitoring และ error handling

### 4. ระบบตรวจสอบสถานะ
- **Health Check**: ทุก 5 วินาที
- **Error Handling**: Auto-restart เมื่อมีข้อผิดพลาดมาก
- **Memory Management**: Buffer cleanup และ monitoring

### 5. ข้อดีของระบบ
- **เสถียร**: Roll Independent (ไม่เปลี่ยนเมื่อเอียงซ้ายขวา)
- **แม่นยำ**: Professional-grade IMU processing
- **เชื่อถือได้**: Real-time quality monitoring
- **ใช้งานง่าย**: Auto-calibration และ error recovery
