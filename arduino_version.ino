// ESP32 + MPU6050 Ultra High Accuracy Pitch Angle
// Arduino IDE Version

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

// สร้าง object สำหรับ MPU6050
MPU6050 mpu;

// ตัวแปรสำหรับข้อมูล MPU6050
int16_t ax, ay, az, gx, gy, gz;
int16_t axOffset = 0, ayOffset = 0, azOffset = 0;
int16_t gxOffset = 0, gyOffset = 0, gzOffset = 0;

// ตัวแปรสำหรับการกรองสัญญาณ
float pitchRaw = 0.0;
float pitchFiltered = 0.0;
float pitchSmooth = 0.0;
float pitchPrev = 0.0;
float pitchUltraSmooth = 0.0;

// ตัวแปรสำหรับการวัดความแม่นยำ
float pitchVariance = 0.0;
float pitchSum = 0.0;
float pitchSumSquare = 0.0;
int sampleCount = 0;

// ตัวแปรเวลา
unsigned long lastTime = 0;
unsigned long calibrationTime = 0;
bool calibrationComplete = false;

void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32 + MPU6050 Ultra High Accuracy Pitch Angle ===");
    
    // เริ่มต้น I2C
    Wire.begin();
    Serial.println("I2C initialized");
    
    // เริ่มต้น MPU6050
    mpu.initialize();
    
    // ตรวจสอบการเชื่อมต่อ
    if (mpu.testConnection()) {
        Serial.println("✅ MPU6050 Connected");
    } else {
        Serial.println("❌ MPU6050 Connection Failed");
        while (1) delay(1000);
    }
    
    // ตั้งค่าความแม่นยำสูง
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);  // ±250°/s
    mpu.setDLPFMode(MPU6050_DLPF_BW_5);             // 5Hz Low Pass Filter
    
    Serial.println("Starting calibration...");
    Serial.println("Please keep ESP32 STABLE during calibration");
    
    calibrationTime = millis();
}

void loop() {
    // ขั้นตอนการคาลิเบรชัน
    if (!calibrationComplete) {
        performCalibration();
        return;
    }
    
    // อ่านข้อมูลจาก MPU6050
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // ปรับค่าด้วย offset
    ax -= axOffset;
    ay -= ayOffset;
    az -= azOffset;
    
    // คำนวณมุม pitch จาก accelerometer
    pitchRaw = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
    
    // กรองสัญญาณ 3 ชั้น
    pitchFiltered = pitchRaw * 0.1 + pitchFiltered * 0.9;        // First-order filter
    pitchSmooth = pitchFiltered * 0.3 + pitchSmooth * 0.7;       // Second-order filter
    pitchUltraSmooth = pitchSmooth * 0.5 + pitchUltraSmooth * 0.5; // Third-order filter
    
    // คำนวณความแปรปรวน
    sampleCount++;
    pitchSum += pitchUltraSmooth;
    pitchSumSquare += pitchUltraSmooth * pitchUltraSmooth;
    
    if (sampleCount >= 100) {
        float mean = pitchSum / sampleCount;
        pitchVariance = (pitchSumSquare / sampleCount) - (mean * mean);
        
        // รีเซ็ตการคำนวณ
        pitchSum = 0.0;
        pitchSumSquare = 0.0;
        sampleCount = 0;
    }
    
    // แสดงผลทุก 0.5 วินาที
    if (millis() - lastTime >= 500) {
        Serial.println("==== PITCH ANGLE MEASUREMENT ====");
        Serial.printf("Raw:         %.3f°\n", pitchRaw);
        Serial.printf("Filtered:    %.3f°\n", pitchFiltered);
        Serial.printf("Smooth:      %.3f°\n", pitchSmooth);
        Serial.printf("Ultra-Smooth: %.3f°\n", pitchUltraSmooth);
        
        // แสดงระดับความแม่นยำ
        String stabilityLevel = "UNKNOWN";
        if (pitchVariance < 0.01) stabilityLevel = "ULTRA-STABLE";
        else if (pitchVariance < 0.05) stabilityLevel = "STABLE";
        else if (pitchVariance < 0.1) stabilityLevel = "MODERATE";
        else stabilityLevel = "UNSTABLE";
        
        Serial.printf("Noise Level: %.4f (Status: %s)\n", pitchVariance, stabilityLevel.c_str());
        Serial.printf("Accuracy:    ±%.2f°\n", sqrt(pitchVariance));
        
        // แสดงข้อมูลดิบเป็นครั้งคราว
        static int debugCounter = 0;
        if (debugCounter++ % 10 == 0) {
            Serial.printf("Raw Data: ax=%d, ay=%d, az=%d\n", ax, ay, az);
        }
        
        Serial.println("===================================");
        
        lastTime = millis();
    }
    
    delay(10);
}

void performCalibration() {
    static long axSum = 0, aySum = 0, azSum = 0;
    static long gxSum = 0, gySum = 0, gzSum = 0;
    static int calibrationSamples = 0;
    
    // อ่านข้อมูลสำหรับคาลิเบรชัน
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    axSum += ax;
    aySum += ay;
    azSum += az;
    gxSum += gx;
    gySum += gy;
    gzSum += gz;
    
    calibrationSamples++;
    
    // แสดงความคืบหน้า
    if (calibrationSamples % 200 == 0) {
        Serial.printf("Calibration progress: %d/2000\n", calibrationSamples);
    }
    
    // เสร็จสิ้นการคาลิเบรชัน
    if (calibrationSamples >= 2000) {
        axOffset = axSum / calibrationSamples;
        ayOffset = aySum / calibrationSamples;
        azOffset = (azSum / calibrationSamples) - 16384; // 1g = 16384 (สำหรับ ±2g)
        gxOffset = gxSum / calibrationSamples;
        gyOffset = gySum / calibrationSamples;
        gzOffset = gzSum / calibrationSamples;
        
        Serial.println("✅ Calibration Complete!");
        Serial.printf("Offsets: ax=%d, ay=%d, az=%d\n", axOffset, ayOffset, azOffset);
        Serial.printf("Gyro Offsets: gx=%d, gy=%d, gz=%d\n", gxOffset, gyOffset, gzOffset);
        Serial.println("Starting measurement...");
        
        calibrationComplete = true;
    }
    
    delay(10);
} 