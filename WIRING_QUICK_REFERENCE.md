# Quick Wiring Reference

## 🔥 CRITICAL CHANGES FROM PREVIOUS VERSION

### **GPIO 5 Now Used for UART**
- **Previous**: Available/unused
- **Now**: TMC2209 UART communication (connects to both drivers' PDN_UART pins)

### **Driver Addresses Required**
You MUST configure AD0/AD1 pins on each TMC2209 board for unique addresses!

---

## 📌 Complete Pin Mapping

### **ESP32 Feather V2 → TMC2209 Driver #1 (Motor 1)**
```
GPIO 26 (A0)  → STEP_1
GPIO 25 (A1)  → DIR_1
GPIO 33 (D33) → EN_1
GPIO 5  (SCK) → PDN_UART (shared with Driver #2)
```

**Driver #1 Addressing:**
```
AD0 → GND
AD1 → GND
(Address = 0b00)
```

---

### **ESP32 Feather V2 → TMC2209 Driver #2 (Motor 2)**
```
GPIO 27 (D27) → STEP_2
GPIO 12 (D12) → DIR_2
GPIO 4  (A5)  → EN_2
GPIO 5  (SCK) → PDN_UART (shared with Driver #1)
```

**Driver #2 Addressing:**
```
AD0 → VIO (+3.3V)
AD1 → GND
(Address = 0b01)
```

---

### **Home Switches**
```
GPIO 34 → HOME_SWITCH_1 (Active LOW, internal pullup)
GPIO 36 → HOME_SWITCH_2 (Active LOW, internal pullup)
```

---

### **Vehicle Magnetic Sensors (Left to Right)**
```
GPIO 19 → Sensor 0 (Far Left)
GPIO 21 → Sensor 1 (Left)
GPIO 7  → Sensor 2 (CENTER) ← Docking target
GPIO 8  → Sensor 3 (Right)
GPIO 37 → Sensor 4 (Far Right)
```

---

### **User Interface**
```
GPIO 13 → Red LED (Active HIGH)
GPIO 15 → Button A (Jog Forward)  - INPUT_PULLUP
GPIO 32 → Button B (Direction/Diagnostics) - INPUT_PULLUP
GPIO 14 → Button C (Jog Reverse)  - INPUT_PULLUP
```

---

### **Display (I2C OLED)**
```
GPIO 20 (SCL) → Display Clock
GPIO 22 (SDA) → Display Data
```

---

## ⚡ Power Supply Wiring

### **12V Power Distribution:**
```
+12V Power Supply → TMC2209 #1 VM+ and TMC2209 #2 VM+
GND Power Supply  → All GND pins (ESP32, both TMC2209s)
```

### **3.3V Logic Level:**
```
ESP32 3V Pin → TMC2209 #1 VIO and TMC2209 #2 VIO
```

**⚠️ WARNING:** Do NOT connect VM+ (12V) to VIO (3.3V) - this will destroy the drivers!

---

## 🧪 Pre-Flight Checklist

Before powering on:

- [ ] Both TMC2209 AD0/AD1 pins configured (Driver 1: both GND, Driver 2: AD0=VIO, AD1=GND)
- [ ] GPIO 5 connected to BOTH drivers' PDN_UART pins
- [ ] All STEP/DIR/EN pins connected
- [ ] Motor coils properly paired (A1-A2 = one coil, B1-B2 = other coil)
- [ ] 12V power connected to VM+ (both drivers)
- [ ] 3.3V connected to VIO (both drivers)
- [ ] Common GND between ESP32 and drivers
- [ ] Home switches connected to GPIO 34 and 36
- [ ] TMC2209Stepper library installed in Arduino IDE

---

## 🧰 First Boot Diagnostics

1. **Upload code**
2. **Open Serial Monitor** (115200 baud)
3. **Watch for:**
   ```
   Starting Can Loader...
   Initializing TMC2209 UART...
   Configuring TMC2209 Drivers...
   Driver 1 configured:
     RMS Current: 600 mA
     Microsteps: 8
     StealthChop: Enabled
   Driver 2 configured:
     [same output]
   
   === TMC2209 Driver Status ===
   [Diagnostic readout]
   ```

4. **If "Open Load" errors appear:**
   - Motor not connected
   - Wrong coil pairing
   - Damaged motor winding

5. **If motors still buzz without rotating:**
   - Press and HOLD Button B → Triggers live diagnostics
   - Check Serial Monitor for error flags
   - Verify motor coil pairing with multimeter

---

## 📈 Speed Tuning (After Motors Work)

Once motors rotate correctly, gradually increase speed in code:

**Current (conservative):**
```cpp
constexpr float STEPPER_MAX_SPEED = 2000.0f;  // ~25 RPM
constexpr float STEPPER_ACCEL = 1000.0f;
```

**Target (high performance):**
```cpp
constexpr float STEPPER_MAX_SPEED = 15000.0f; // 110 RPM
constexpr float STEPPER_ACCEL = 8000.0f;
```

Increase gradually: 2000 → 5000 → 10000 → 15000, testing at each step.
