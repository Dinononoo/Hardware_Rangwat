/**
  GY-25 MPU6050 Module for ESP32
  Improved version with better error handling and stability
  
  ✅ ESP32 Features:
  - Hardware Serial support (Serial2)
  - Better performance and stability
  - WiFi integration ready
  - Enhanced debugging
*/

#include <Arduino.h>

// ================ Pin Configuration ================
#define GY25_RX_PIN 16  // GPIO16 - ESP32 RX2
#define GY25_TX_PIN 17  // GPIO17 - ESP32 TX2

// ================ Global Variables ================
float Roll, Pitch, Yaw;
float Roll_filtered = 0, Pitch_filtered = 0, Yaw_filtered = 0;
unsigned char buffer[8], counter = 0;
bool imu_updated = false;

// ✅ เพิ่มตัวแปรสำหรับการจัดการข้อผิดพลาด
bool gy25_connected = false;
bool gy25_healthy = false;
unsigned long last_data_time = 0;
unsigned long last_health_check = 0;
unsigned long last_calibration = 0;
int error_count = 0;
const int max_errors = 10;

// ✅ เพิ่มตัวแปรสำหรับการกรองข้อมูล
const float filter_alpha = 0.3;  // Low-pass filter coefficient
const unsigned long data_timeout = 2000;  // 2 seconds timeout
const unsigned long health_check_interval = 5000;  // 5 seconds
const unsigned long calibration_interval = 300000;  // 5 minutes

// ✅ เพิ่มตัวแปรสำหรับการแสดงผล
unsigned long last_display_time = 0;
const unsigned long display_interval = 1000;  // 1 second

// ================ Function Declarations ================
void readGY25Data();

// ================ GY-25 Functions ================
// ✅ เพิ่มฟังก์ชันตรวจสอบสถานะ GY-25
bool checkGY25Health() {
    unsigned long current_time = millis();
    
    // ตรวจสอบว่ามีข้อมูลใหม่หรือไม่
    if (current_time - last_data_time > data_timeout) {
        gy25_healthy = false;
        return false;
    }
    
    // ตรวจสอบค่าที่สมเหตุสมผล
    if (abs(Roll) > 180 || abs(Pitch) > 180 || abs(Yaw) > 360) {
        gy25_healthy = false;
        return false;
    }
    
    gy25_healthy = true;
    return true;
}

// ✅ เพิ่มฟังก์ชันรีเซ็ต GY-25
void resetGY25() {
    Serial.println("🔄 Resetting GY-25...");
    
    // ส่งคำสั่งรีเซ็ต
    Serial2.write(0xA5);
    Serial2.write(0x54);  // Correction mode
    delay(1000);
    
    Serial2.write(0xA5);
    Serial2.write(0x52);  // Automatic mode
    delay(1000);
    
    // รีเซ็ตตัวแปร
    counter = 0;
    error_count = 0;
    last_data_time = millis();
    
    Serial.println("✅ GY-25 Reset Complete!");
}

// ✅ เพิ่มฟังก์ชันกรองข้อมูล
void applyFilter() {
    Roll_filtered = filter_alpha * Roll + (1 - filter_alpha) * Roll_filtered;
    Pitch_filtered = filter_alpha * Pitch + (1 - filter_alpha) * Pitch_filtered;
    Yaw_filtered = filter_alpha * Yaw + (1 - filter_alpha) * Yaw_filtered;
}

// ✅ เพิ่มฟังก์ชันแสดงสถานะ
void displayStatus() {
    unsigned long current_time = millis();
    
    if (current_time - last_display_time >= display_interval) {
        Serial.println("📐 GY-25 Status:");
        Serial.printf("   🧮 Roll: %+7.2f° (Filtered: %+7.2f°)\n", Roll, Roll_filtered);
        Serial.printf("   📐 Pitch: %+7.2f° (Filtered: %+7.2f°)\n", Pitch, Pitch_filtered);
        Serial.printf("   📐 Yaw: %+7.2f° (Filtered: %+7.2f°)\n", Yaw, Yaw_filtered);
        
        // แสดงสถานะการเชื่อมต่อ
        if (gy25_healthy) {
            Serial.println("   ✅ GY-25: Healthy and Connected");
        } else {
            Serial.println("   ⚠️ GY-25: Connection Issues");
        }
        
        // แสดงจำนวนข้อผิดพลาด
        if (error_count > 0) {
            Serial.printf("   ⚠️ Errors: %d/%d\n", error_count, max_errors);
        }
        
        // แสดงสถานะหน่วยความจำ ESP32
        Serial.printf("   💾 Free Heap: %d bytes\n", ESP.getFreeHeap());
        
        last_display_time = current_time;
    }
}

// ✅ เพิ่มฟังก์ชันตรวจสอบข้อมูล
bool validateData() {
    // ตรวจสอบค่าที่สมเหตุสมผล
    if (abs(Roll) > 180 || abs(Pitch) > 180 || abs(Yaw) > 360) {
        return false;
    }
    
    // ตรวจสอบการเปลี่ยนแปลงที่ผิดปกติ
    static float last_roll = 0, last_pitch = 0, last_yaw = 0;
    float roll_diff = abs(Roll - last_roll);
    float pitch_diff = abs(Pitch - last_pitch);
    float yaw_diff = abs(Yaw - last_yaw);
    
    if (roll_diff > 90 || pitch_diff > 90 || yaw_diff > 180) {
        return false;  // การเปลี่ยนแปลงผิดปกติ
    }
    
    last_roll = Roll;
    last_pitch = Pitch;
    last_yaw = Yaw;
    
    return true;
}

// ================ Setup Function ================
void setup() {
    Serial.begin(115200);
    
    // ✅ ใช้ Hardware Serial สำหรับ ESP32
    Serial2.begin(9600, SERIAL_8N1, GY25_RX_PIN, GY25_TX_PIN);
    
    delay(1000);
    Serial.println("🎯 GY-25 MPU6050 Module for ESP32");
    Serial.println("=====================================");
    
    // ✅ ปรับปรุงการตั้งค่า GY-25
    Serial.println("🔧 Setting up GY-25...");
    
    // Step 1: Correction mode (pitch/roll correction)
    Serial2.write(0xA5);
    Serial2.write(0x54);
    Serial.println("   📐 Calibrating GY-25 (pitch/roll correction)");
    Serial.println("   ⚠️ Keep module level and still for 4 seconds...");
    delay(4000);
    
    // Step 2: Set to automatic mode
    Serial2.write(0xA5);
    Serial2.write(0x52);  // Automatic mode
    Serial.println("   ✅ GY-25 set to automatic mode");
    
    // Step 3: Initialize variables
    last_data_time = millis();
    last_health_check = millis();
    last_calibration = millis();
    
    Serial.println("✅ GY-25 Setup Complete!");
    Serial.println("📊 Ready to receive angle data...");
    Serial.printf("🔌 Using ESP32 Serial2: RX=GPIO%d, TX=GPIO%d\n", GY25_RX_PIN, GY25_TX_PIN);
    Serial.println();
}

// ================ Main Loop ================
void loop() {
    // ✅ เพิ่มการตรวจสอบสถานะระบบ
    unsigned long current_time = millis();
    
    // ตรวจสอบสถานะ GY-25
    if (current_time - last_health_check >= health_check_interval) {
        if (!checkGY25Health()) {
            error_count++;
            Serial.printf("⚠️ GY-25 Health Check Failed! Error Count: %d/%d\n", error_count, max_errors);
            
            if (error_count >= max_errors) {
                Serial.println("🔄 Too Many Errors - Resetting GY-25...");
                resetGY25();
            }
        } else {
            error_count = 0;  // รีเซ็ตตัวนับเมื่อระบบปกติ
        }
        last_health_check = current_time;
    }
    
    // ✅ เพิ่มการคาลิเบรตเป็นระยะ
    if (current_time - last_calibration >= calibration_interval) {
        Serial.println("🔄 Periodic GY-25 Calibration...");
        Serial2.write(0xA5);
        Serial2.write(0x54);  // Correction mode
        delay(1000);
        Serial2.write(0xA5);
        Serial2.write(0x52);  // Back to automatic mode
        last_calibration = current_time;
    }
    
    // อ่านข้อมูลจาก GY-25
    readGY25Data();
    
    // ประมวลผลข้อมูล
    if (imu_updated) {
        imu_updated = false;
        
        // ✅ ตรวจสอบข้อมูลก่อนประมวลผล
        if (validateData()) {
            last_data_time = current_time;
            applyFilter();
            
            // แสดงผลข้อมูล
            displayStatus();
        } else {
            Serial.println("⚠️ Invalid GY-25 data detected - skipping");
            error_count++;
        }
    }
    
    // ✅ เพิ่มการแสดงผลแบบย่อ
    static unsigned long last_compact_display = 0;
    if (current_time - last_compact_display >= 5000) {  // ทุก 5 วินาที
        Serial.printf("📐 GY-25: R=%+.1f° P=%+.1f° Y=%+.1f° [%s] [Heap: %d]\n", 
                      Roll_filtered, Pitch_filtered, Yaw_filtered,
                      gy25_healthy ? "OK" : "ERROR", ESP.getFreeHeap());
        last_compact_display = current_time;
    }
    
    // ✅ เพิ่มการจัดการ Watchdog Timer
    yield();
    delay(10);
}

// ================ GY-25 Data Reading Function ================
void readGY25Data() {
    while (Serial2.available()) {
        buffer[counter] = (unsigned char)Serial2.read();
        
        // ✅ ปรับปรุงการตรวจสอบข้อมูล
        if (counter == 0 && buffer[0] != 0xAA) {
            // รีเซ็ต counter เมื่อไม่พบ preamble
            counter = 0;
            return;
        }
        
        counter++;
        
        if (counter == 8) {  // package is complete
            counter = 0;
            
            // ✅ ตรวจสอบข้อมูลที่สมบูรณ์
            if (buffer[0] == 0xAA && buffer[7] == 0x55) {  // data package is correct
                // ✅ ปรับปรุงการคำนวณมุม
                int16_t yaw_raw = (int16_t)(buffer[1] << 8 | buffer[2]);
                int16_t pitch_raw = (int16_t)(buffer[3] << 8 | buffer[4]);
                int16_t roll_raw = (int16_t)(buffer[5] << 8 | buffer[6]);
                
                // แปลงเป็นองศา
                Yaw = yaw_raw / 100.0;
                Pitch = pitch_raw / 100.0;
                Roll = roll_raw / 100.0;
                
                // ✅ ตรวจสอบค่าที่สมเหตุสมผล
                if (abs(Roll) <= 180 && abs(Pitch) <= 180 && abs(Yaw) <= 360) {
                    imu_updated = true;
                    gy25_connected = true;
                } else {
                    Serial.println("⚠️ GY-25: Invalid angle values detected");
                    error_count++;
                }
            } else {
                // ✅ นับข้อผิดพลาดเมื่อข้อมูลไม่ถูกต้อง
                Serial.println("⚠️ GY-25: Invalid data packet");
                error_count++;
            }
        }
    }
}
