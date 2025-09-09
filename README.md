# ESP32 + MPU6050 Basic Project

โปรเจกต์พื้นฐานสำหรับการเชื่อมต่อและทดสอบ ESP32 กับเซ็นเซอร์ MPU6050

## 🔧 Hardware Requirements

- **ESP32 Development Board** (ESP32-WROOM-32)
- **MPU6050 Sensor Module** (6-axis Motion Tracking)
- **Jumper Wires** (4 เส้น)
- **Breadboard** (ไม่บังคับ)

## 📋 การต่อสายระหว่าง ESP32 และ MPU6050

### **การเชื่อมต่อ:**
```
ESP32 Pin          →          MPU6050 Pin
================================================
3.3V               →          VCC
GND                →          GND
GPIO21 (SDA)       →          SDA
GPIO22 (SCL)       →          SCL
```

### **รายละเอียดสัญญาณ:**
- **VCC**: แรงดันไฟฟ้า 3.3V จาก ESP32
- **GND**: สายกราวด์ (ลบ) ต่อกัน
- **SDA**: Serial Data Line (สายข้อมูล I2C)
- **SCL**: Serial Clock Line (สายนาฬิกา I2C)

## 🛠️ การติดตั้งและใช้งาน

### 1. ติดตั้ง PlatformIO
```bash
# ติดตั้ง PlatformIO Core
pip install platformio

# หรือใช้ VS Code Extension
# ติดตั้ง PlatformIO IDE for VS Code
```

### 2. Build และ Upload
```bash
# เข้าไปในโฟลเดอร์โปรเจกต์
cd Rangwat

# Build โปรเจกต์
pio run

# Upload ไปยัง ESP32
pio run --target upload

# ดู Serial Monitor
pio device monitor
```

## 📊 ผลลัพธ์ที่คาดหวัง

เมื่อรันโปรแกรมสำเร็จ คุณจะเห็นผลลัพธ์ใน Serial Monitor:

```
🚀 ESP32 + MPU6050 Basic Test
🔧 Initialize MPU6050...
✅ MPU6050 Connected
📊 Starting to read sensor data...
Format: Axyz [ax] [ay] [az] Gxyz [gx] [gy] [gz]
===============================================
Axyz    90      85      95      Gxyz    88      92      90
Raw: A(-1250,2340,-850) G(456,-123,789)
---
Axyz    91      86      94      Gxyz    89      91      91
Raw: A(-1180,2380,-820) G(467,-134,798)
---
```

## 🔍 การทำงานของโปรแกรม

### **ข้อมูลที่แสดง:**
- **Axyz**: ค่า Accelerometer แกน X, Y, Z (แปลงเป็น 0-180 องศา)
- **Gxyz**: ค่า Gyroscope แกน X, Y, Z (แปลงเป็น 0-180 องศา)
- **Raw**: ค่าดิบจากเซ็นเซอร์ (ก่อนแปลง)

### **การแปลงค่า:**
```cpp
// แปลงค่าดิบ (-18000 ถึง +18000) เป็นองศา (0-180)
int16_t ax_mapped = map(ax, -18000, 18000, 0, 180);
```

## 🧪 การทดสอบ

### **ทดสอบการเชื่อมต่อ:**
1. Upload โปรแกรม
2. เปิด Serial Monitor
3. ต้องเห็นข้อความ "✅ MPU6050 Connected"

### **ทดสอบการอ่านข้อมูล:**
1. เอียง/หมุน ESP32 + MPU6050
2. ดูค่าที่เปลี่ยนแปลงใน Serial Monitor
3. ค่า Accelerometer จะเปลี่ยนตามทิศทาง
4. ค่า Gyroscope จะเปลี่ยนตามการหมุน

## 🚨 Troubleshooting

### **ปัญหาที่พบบ่อย:**

#### 1. "❌ MPU6050 Connection failed"
**สาเหตุ:** การต่อสายผิด หรือ MPU6050 เสีย
**แก้ไข:**
- ตรวจสอบการต่อสาย VCC, GND, SDA, SCL
- ลองใช้ MPU6050 ตัวอื่น
- ตรวจสอบว่า ESP32 ได้ไฟเลี้ยง

#### 2. ไม่มีข้อมูลแสดงใน Serial Monitor
**สาเหตุ:** Serial Monitor ตั้งค่าผิด
**แก้ไข:**
- ตั้งค่า Baud Rate เป็น 115200
- ตรวจสอบ COM Port ถูกต้อง

#### 3. ค่าไม่เปลี่ยนแปลง
**สาเหตุ:** เซ็นเซอร์ไม่ทำงาน
**แก้ไข:**
- ลองเขย่าหรือเอียง ESP32
- ตรวจสอบการต่อสาย SDA, SCL
- ตรวจสอบว่า MPU6050 ได้ไฟเลี้ยง

## 📚 ขั้นตอนต่อไป

เมื่อโปรแกรมพื้นฐานทำงานได้แล้ว คุณสามารถ:
1. **เพิ่ม LED** แสดงสถานะ
2. **เพิ่ม BLE** ส่งข้อมูลไปมือถือ
3. **คำนวณมุม** pitch, roll, yaw
4. **เพิ่มการกรองสัญญาณ** ลดสัญญาณรบกวน
5. **บันทึกข้อมูล** ลง SD Card

## 📞 Support

หากมีปัญหา สามารถอ้างอิง:
- [ESP32 Pinout Reference](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/)
- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [I2C Protocol](https://learn.sparkfun.com/tutorials/i2c) 