# TMC2209 UART Upgrade - Complete Checklist

## 📦 Step 1: Install Library (REQUIRED)

- [ ] Install **TMCStepper by teemuatlut** via Arduino Library Manager
- [ ] Restart Visual Studio
- [ ] Verify `TMC2209Stepper.h` compiles without errors
- [ ] See `INSTALL_TMC2209_LIBRARY.md` for detailed instructions

---

## 🔌 Step 2: Hardware Modifications

### **A. Configure Driver Addresses**
- [ ] **Driver #1**: Solder/jumper AD0 → GND
- [ ] **Driver #1**: Solder/jumper AD1 → GND
- [ ] **Driver #2**: Solder/jumper AD0 → VIO
- [ ] **Driver #2**: Solder/jumper AD1 → GND
- [ ] Verify with multimeter (see `TMC2209_ADDRESSING_GUIDE.md`)

### **B. Add UART Wiring**
- [ ] Connect **ESP32 GPIO 5** to **Driver #1 PDN_UART**
- [ ] Connect **ESP32 GPIO 5** to **Driver #2 PDN_UART** (Y-split or breadboard)
- [ ] Keep wire length <15cm for reliability

### **C. Verify Existing Wiring (No Changes)**
**Motor 1 (Driver #1):**
- [ ] STEP: GPIO 26 → Driver #1 STEP
- [ ] DIR: GPIO 25 → Driver #1 DIR
- [ ] EN: GPIO 33 → Driver #1 EN

**Motor 2 (Driver #2):**
- [ ] STEP: GPIO 27 → Driver #2 STEP
- [ ] DIR: GPIO 12 → Driver #2 DIR
- [ ] EN: GPIO 4 → Driver #2 EN

**Power:**
- [ ] +12V → Both drivers' VM+
- [ ] +3.3V (ESP32 3V pin) → Both drivers' VIO
- [ ] GND → Common ground (ESP32 + both drivers + power supply)

**Motors:**
- [ ] Verify coil pairs with multimeter (~2-10Ω between coil wires)
- [ ] Coil A → A1/A2 terminals
- [ ] Coil B → B1/B2 terminals

---

## 💻 Step 3: Upload Code

- [ ] Open Serial Monitor (115200 baud)
- [ ] Upload code to ESP32
- [ ] Watch for initialization messages

---

## 🧪 Step 4: Verify Communication

### **Expected Serial Output:**
```
Starting Can Loader...
Initializing TMC2209 UART...
Configuring TMC2209 Drivers...
Configuring TMC2209 via UART...
Driver 1 configured:
  RMS Current: 600 mA       ← MUST see this!
  Microsteps: 8
  StealthChop: Enabled
Driver 2 configured:
  RMS Current: 600 mA       ← MUST see this!
  Microsteps: 8
  StealthChop: Enabled

=== TMC2209 Driver Status ===
[Diagnostic readout]
```

### **✅ Success Indicators:**
- [ ] Both drivers show "600 mA" current
- [ ] Both show "Microsteps: 8"
- [ ] DRV_STATUS shows valid hex value (not 0x00000000)
- [ ] "Open Load" flags are **NO** (if YES = motor wiring problem)

### **❌ Failure Indicators:**
- [ ] "RMS Current: 0 mA" → Address conflict or UART wiring issue
- [ ] "Open Load A/B: YES" → Motor coils not connected or wrong pairing
- [ ] "Short to GND: YES" → Wiring short, check for crossed wires

---

## 🎬 Step 5: Test Homing

After initialization, homing should start automatically:

- [ ] **Motors rotate smoothly** towards home switches (not just buzzing!)
- [ ] Serial Monitor: "Home switch triggered"
- [ ] Motors stop and back off 50 microsteps
- [ ] Display shows: "Homed" → "Waiting"

### **If motors buzz but don't move:**
1. Press and **HOLD Button B** (1+ seconds) → Prints diagnostics
2. Check Serial Monitor for error flags
3. Common issues:
   - **Open Load**: Motor coils swapped (A↔B), try reversing all 4 wires
   - **StallGuard = 0**: Motor mechanically jammed
   - **No communication**: Check GPIO 5 → PDN_UART connections

---

## 🕹️ Step 6: Test Manual Jogging

After homing completes:

- [ ] Press **Button A** → Motors extend (should rotate forward)
- [ ] Press **Button C** → Motors retract (should rotate backward)
- [ ] Motion should be **smooth and nearly silent** (StealthChop)

### **Expected Performance (Conservative Settings):**
- Speed: ~25 RPM (2000 microsteps/sec)
- Noise: Very quiet, slight hum
- Movement: Smooth, no jerking

---

## 🚀 Step 7: Performance Tuning

Once basic operation works, increase speed in code:

### **Edit `Can Loader.ino`:**
```cpp
// Change these lines (around line 180):
constexpr float STEPPER_MAX_SPEED = 15000.0f; // Was 2000.0f
constexpr float STEPPER_ACCEL = 8000.0f;      // Was 1000.0f
```

### **Re-upload and test:**
- [ ] Full load cycle should complete in ~5 seconds (vs 15+ seconds baseline)
- [ ] Motors should still be quiet (if loud, reduce to 10000)
- [ ] No missed steps (watch for jittery motion)

---

## 🔧 Step 8: Optional Enhancements

### **A. Sensorless Homing (Remove Limit Switches)**
Once UART works, StallGuard can detect when motor hits mechanical stop:
- Eliminates need for HOME_SWITCH_1 and HOME_SWITCH_2
- Frees up GPIO 34 and 36 for other uses
- More reliable (no switch bouncing)

### **B. Temperature Monitoring**
Add to display:
```cpp
int temp1 = driver1.TSTEP();  // Read temperature register
display.print("T:"); display.print(temp1); display.print("C");
```

### **C. Auto-Recovery from Errors**
Monitor for over-temp and automatically reduce speed/current

---

## 📊 Troubleshooting Decision Tree

```
Motors buzz but don't rotate?
├─ Open Serial Monitor
│  ├─ "RMS Current: 0 mA"?
│  │  ├─ YES → Check UART wiring (GPIO 5 → PDN_UART)
│  │  │        Check driver addressing (AD0/AD1)
│  │  └─ NO → Continue...
│  │
│  ├─ "Open Load A/B: YES"?
│  │  ├─ YES → Motor coils not paired correctly
│  │  │        Use multimeter to find coil pairs
│  │  │        Try swapping A↔B on one motor
│  │  └─ NO → Continue...
│  │
│  ├─ "StallGuard Result: 0"?
│  │  ├─ YES → Motor mechanically jammed
│  │  │        Disconnect motor from gearbox, test alone
│  │  └─ NO → Continue...
│  │
│  └─ Check motor coil resistance (~2-10Ω per coil)
│     If open circuit → Bad motor winding
│
└─ If still stuck → Post diagnostics output in forum/issue tracker
```

---

## 🎉 Success Criteria

### **You're done when:**
1. ✅ Serial Monitor shows "600 mA" for both drivers
2. ✅ Homing completes successfully (motors rotate to switches)
3. ✅ Jog buttons move motors smoothly and quietly
4. ✅ Full load cycle completes in <10 seconds
5. ✅ Button B long-press shows live diagnostics

### **Expected Final Performance:**
- **Speed**: 110 RPM (15,000 microsteps/sec)
- **Load Time**: 5 seconds (84mm travel)
- **Noise**: Nearly silent, StealthChop active
- **Heat**: Minimal, CoolStep reduces idle current
- **Reliability**: Live error monitoring, automatic recovery

---

## 📞 Getting Help

If stuck, provide this info:
1. Serial Monitor output (full boot sequence)
2. Button B diagnostic output (hold 1+ sec)
3. Photo of TMC2209 wiring (especially AD0/AD1)
4. Multimeter readings: VM+ voltage, VIO voltage, AD0/AD1 voltages

Good luck! The UART diagnostics will reveal exactly why the motors are buzzing. 🔍
