# TMC2209 Library Installation

## ⚠️ REQUIRED: Install TMC2209Stepper Library

Your code needs the `TMC2209Stepper` library for UART communication with the drivers.

---

## Installation Methods

### **Method 1: Arduino Library Manager (Recommended)**

1. Open **Tools → Manage Libraries...** in Arduino IDE
2. Search for: **"TMC2209Stepper"**
3. Install: **"TMCStepper by teemuatlut"**
   - This is the official library that includes TMC2209 support
   - Latest version: 0.7.3 or newer
4. Restart Visual Studio/Arduino IDE

---

### **Method 2: Manual Installation**

If Library Manager doesn't work:

1. Download from GitHub:
   ```
   https://github.com/teemuatlut/TMCStepper
   ```

2. Extract to your Arduino libraries folder:
   ```
   C:\Users\grayl\Documents\Arduino\libraries\TMCStepper\
   ```

3. Restart Visual Studio

---

### **Method 3: PlatformIO (If using PlatformIO)**

Add to `platformio.ini`:
```ini
lib_deps = 
    teemuatlut/TMCStepper@^0.7.3
```

---

## ✅ Verify Installation

After installing, check that these files exist:
```
Arduino\libraries\TMCStepper\src\TMC2209Stepper.h
Arduino\libraries\TMCStepper\src\TMCStepper.h
```

If the files exist, your code should compile successfully!

---

## 🔧 After Library Installation

1. **Close and reopen Visual Studio** (refresh IntelliSense)
2. **Rebuild the project**
3. Upload to ESP32
4. Watch Serial Monitor for driver diagnostics during startup

---

## 📖 Library Documentation

GitHub: https://github.com/teemuatlut/TMCStepper
Wiki: https://github.com/teemuatlut/TMCStepper/wiki

The library provides:
- Full TMC2209 register access
- StealthChop/SpreadCycle control
- CoolStep configuration
- StallGuard (sensorless homing)
- Real-time diagnostics
