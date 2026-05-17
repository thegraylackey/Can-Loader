# TMC2209 UART Control - Wiring Guide

## 🔌 Hardware Setup

### **Required Hardware Modifications**

#### **1. UART Connection**
Connect **GPIO 5** to both TMC2209 boards' PDN_UART pins:
```
ESP32 GPIO 5 (PIN_SCK_5) ──┬── TMC2209 #1 PDN_UART pin
                            └── TMC2209 #2 PDN_UART pin
```

**Note:** Single-wire bidirectional communication - same pin for TX and RX.

---

### **2. Driver Addressing (CRITICAL)**

Each TMC2209 must have a **unique UART address**. Configure using the AD0 and AD1 pins on each board:

#### **TMC2209 Driver #1 (Motor 1):**
- **AD0**: Connect to **GND**
- **AD1**: Connect to **GND**
- **Address**: 0b00 (Address 0)

#### **TMC2209 Driver #2 (Motor 2):**
- **AD0**: Connect to **VIO** (3.3V)
- **AD1**: Connect to **GND**
- **Address**: 0b01 (Address 1)

---

### **3. Complete TMC2209 Wiring**

#### **Driver #1 (Motor 1) Connections:**
| TMC2209 Pin | Connect To | ESP32 Pin | Notes |
|-------------|------------|-----------|-------|
| VM+         | +12V       | -         | Motor power supply |
| GND         | GND        | -         | Common ground with ESP32 |
| VIO         | +3.3V      | 3V        | Logic level (from ESP32) |
| EN          | GPIO 33    | D33       | Active LOW enable |
| STEP        | GPIO 26    | A0        | Step pulse signal |
| DIR         | GPIO 25    | A1        | Direction signal |
| PDN_UART    | GPIO 5     | SCK       | UART communication (shared) |
| **AD0**     | **GND**    | -         | **Address bit 0** |
| **AD1**     | **GND**    | -         | **Address bit 1** |
| MS1         | Float      | -         | Microstepping (controlled via UART) |
| MS2         | Float      | -         | Microstepping (controlled via UART) |
| A1, A2      | Motor Coil A | -       | Motor phase A |
| B1, B2      | Motor Coil B | -       | Motor phase B |

#### **Driver #2 (Motor 2) Connections:**
| TMC2209 Pin | Connect To | ESP32 Pin | Notes |
|-------------|------------|-----------|-------|
| VM+         | +12V       | -         | Motor power supply |
| GND         | GND        | -         | Common ground with ESP32 |
| VIO         | +3.3V      | 3V        | Logic level (from ESP32) |
| EN          | GPIO 4     | A5        | Active LOW enable |
| STEP        | GPIO 27    | D27       | Step pulse signal |
| DIR         | GPIO 12    | D12       | Direction signal (BOOT pin - OK) |
| PDN_UART    | GPIO 5     | SCK       | UART communication (shared) |
| **AD0**     | **VIO (+3.3V)** | -    | **Address bit 0** |
| **AD1**     | **GND**    | -         | **Address bit 1** |
| MS1         | Float      | -         | Microstepping (controlled via UART) |
| MS2         | Float      | -         | Microstepping (controlled via UART) |
| A1, A2      | Motor Coil A | -       | Motor phase A |
| B1, B2      | Motor Coil B | -       | Motor phase B |

---

## 📋 Motor Wiring Check

### **IMPORTANT: Verify Coil Pairing**

If motors buzz but don't rotate, you may have incorrect coil pairing. Use a multimeter in continuity/resistance mode:

1. **Disconnect motor from driver**
2. **Find coil pairs** - measure resistance between motor wires:
   - Coil A: Two wires with ~2-10Ω resistance between them
   - Coil B: The OTHER two wires with ~2-10Ω resistance
   - No continuity between coils

3. **Connect to TMC2209:**
   - Coil A → A1 and A2 terminals
   - Coil B → B1 and B2 terminals

4. **If motor spins backwards** - swap both coil pairs (A↔B)

---

## ⚙️ Software Configuration (Already Done!)

The code now automatically configures via UART:
- ✅ Motor current: 600mA RMS (no VREF adjustment needed!)
- ✅ Microstepping: 1/8 step
- ✅ StealthChop: Enabled (silent operation)
- ✅ CoolStep: Enabled (dynamic current reduction)
- ✅ StallGuard: Configured for future sensorless homing

---

## 🛠️ Troubleshooting the Buzzing Issue

### **After wiring UART and uploading new code:**

1. **Open Serial Monitor** at 115200 baud
2. Watch for driver initialization messages
3. **Look for error flags** in driver status:
   - `Open Load A/B: YES` → Motor not connected or wrong wiring
   - `Short to GND: YES` → Wiring short or damaged driver
   - `Over Temp: YES` → Driver overheating (check current setting)
   - `StallGuard Result: 0` → Motor stalled/jammed

4. **Press and HOLD Button B** for 1+ seconds anytime to print live diagnostics

### **Expected Normal Output:**
```
=== TMC2209 Driver Status ===

--- Driver 1 ---
DRV_STATUS: 0xC0000000
  StallGuard Result: 150
  Standstill: YES
  Open Load A: NO
  Open Load B: NO
  Short to GND A: NO
  Short to GND B: NO
  Over Temp Pre-warn: NO
  Over Temp: NO

--- Driver 2 ---
[Similar healthy output]
```

### **If Still Buzzing:**
- Check motor coil pairing (see above)
- Verify VM+ has 12V power
- Check all GND connections are solid
- Try swapping A↔B coils on one motor

---

## 🚀 Next Steps After Fixing Buzzing

Once motors rotate correctly, you can:
1. **Increase speed** - Change `STEPPER_MAX_SPEED` from 2000 to 15000 (110 RPM)
2. **Enable sensorless homing** - Use StallGuard instead of limit switches
3. **Monitor temperature** - Display driver temp on OLED during operation
4. **Optimize CoolStep** - Fine-tune current reduction for cooler operation

---

## 📊 Performance Targets

With UART control working:
- **Speed**: 110 RPM (vs 40 RPM baseline) = **2.75x improvement**
- **Load Cycle**: ~5 seconds (vs 15 seconds) = **3x faster**
- **Noise**: Nearly silent (StealthChop mode)
- **Heat**: 50-75% reduction (CoolStep dynamic current)
- **Reliability**: Live error detection and recovery
