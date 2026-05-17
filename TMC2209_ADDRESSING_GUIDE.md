# TMC2209 Addressing Setup - Visual Guide

## 🎯 Critical Configuration: Driver Addresses

Each TMC2209 board has **AD0** and **AD1** pins that set its UART address. Both drivers share the same UART wire (GPIO 5), so they MUST have unique addresses.

---

## 📍 Physical Board Layout

### **TMC2209 V1.3 Board - Top View:**
```
┌─────────────────────────────────────────┐
│  TMC2209 V1.3 Stepper Driver            │
│                                          │
│  [Motor Connections]                     │
│   B2  B1  A1  A2                         │
│   ⚫  ⚫  ⚫  ⚫                         │
│                                          │
│  [Logic Pins - Bottom Edge]              │
│  EN  MS1 MS2 MS2 PDN VIO VM GND         │
│  ⚫   ⚫  ⚫  ⚫  ⚫  ⚫  ⚫  ⚫        │
│                                          │
│  [Address Pins - Small pads/holes]       │
│           AD0   AD1                      │
│            ⚫    ⚫                       │
│         ┌──┘    └──┐                    │
│         │           │                    │
│   (Configure these for addressing)       │
│                                          │
│  [VREF Potentiometer] ← NOT NEEDED!     │
│         🎛️ (UART sets current)          │
└─────────────────────────────────────────┘
```

---

## ⚙️ Driver #1 Configuration (Address 0b00)

### **AD0 and AD1 Setup:**
```
TMC2209 Driver #1
─────────────────
AD0 ──────→ GND
AD1 ──────→ GND

Result: Address = 0b00 (Binary) = 0 (Decimal)
```

### **Wiring Options:**
1. **Solder bridge**: Short AD0 pad to nearby GND pad
2. **Jumper wire**: Connect AD0 pin header to GND pin
3. **Leave floating if board defaults to GND** (some boards have pull-down resistors)

**Test:** If both AD0 and AD1 read LOW with multimeter → Address 0 ✅

---

## ⚙️ Driver #2 Configuration (Address 0b01)

### **AD0 and AD1 Setup:**
```
TMC2209 Driver #2
─────────────────
AD0 ──────→ VIO (+3.3V)
AD1 ──────→ GND

Result: Address = 0b01 (Binary) = 1 (Decimal)
```

### **Wiring Options:**
1. **Solder bridge**: Short AD0 to VIO pad, AD1 to GND pad
2. **Jumper wire**: Connect AD0 to VIO pin, AD1 to GND pin

**Test:** 
- AD0 should read 3.3V with multimeter
- AD1 should read 0V (GND)
- → Address 1 ✅

---

## 🔍 Address Truth Table

| Driver | AD1 | AD0 | Binary | Decimal | Code Variable |
|--------|-----|-----|--------|---------|---------------|
| #1     | GND | GND | 0b00   | 0       | `DRIVER_ADDRESS_1 = 0b00` |
| #2     | GND | VIO | 0b01   | 1       | `DRIVER_ADDRESS_2 = 0b01` |
| (Unused) | VIO | GND | 0b10 | 2 | - |
| (Unused) | VIO | VIO | 0b11 | 3 | - |

---

## 🧪 Testing Address Configuration

### **Before Power-On:**
Use multimeter to verify:
```
Driver #1:
  AD0 to GND: < 0.3V  ✅
  AD1 to GND: < 0.3V  ✅

Driver #2:
  AD0 to VIO: ~3.3V  ✅
  AD1 to GND: < 0.3V  ✅
```

### **After Power-On (with code uploaded):**
Serial Monitor should show:
```
Configuring TMC2209 via UART...
Driver 1 configured:
  RMS Current: 600 mA    ← Should see this!
  Microsteps: 8
  StealthChop: Enabled

Driver 2 configured:
  RMS Current: 600 mA    ← Should see this!
  Microsteps: 8
  StealthChop: Enabled
```

**If you see "0 mA" or no response:**
- Address configuration is wrong
- UART wiring issue
- Driver not powered

---

## 🚨 Common Mistakes

### ❌ **Both Drivers Set to Same Address**
**Symptom:** Only one driver responds, other is silent
**Fix:** Verify AD0/AD1 pins are configured differently

### ❌ **AD Pins Left Floating**
**Symptom:** Random address, unreliable communication
**Fix:** Explicitly connect to GND or VIO

### ❌ **VIO Not Connected**
**Symptom:** No UART communication, drivers appear dead
**Fix:** Connect 3.3V from ESP32 to both drivers' VIO pins

### ❌ **UART Pin Not Connected**
**Symptom:** All register reads return 0, no configuration applied
**Fix:** Check GPIO 5 → PDN_UART wire on BOTH drivers

---

## 🎯 Final Connection Summary

```
ESP32 GPIO 5 ──┬─── Driver #1 PDN_UART (AD0=GND, AD1=GND)
               │
               └─── Driver #2 PDN_UART (AD0=VIO, AD1=GND)
```

**Physical implementation:**
- Use a Y-splitter or breadboard to connect GPIO 5 to both drivers
- Keep wires short (<15cm) for reliable communication
- Twist UART wires with GND if possible (noise reduction)

---

## ✅ Success Indicators

After proper addressing and UART wiring:
1. **Serial Monitor** shows driver configuration (600mA, 8 microsteps)
2. **Motors rotate smoothly** during homing (not just buzzing)
3. **Button B long-press** displays live diagnostics
4. **StallGuard values** change during motion (check diagnostics)

If all three work → You're ready to increase speed to 110 RPM! 🚀
