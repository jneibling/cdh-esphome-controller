# CDH Controller - Wiring & Hardware Guide
## ESP32 Read/Write Controller for Chinese Diesel Heaters with OEM Failover

---

## Overview

This design adds **full wireless control** of your Chinese diesel heater via ESP32 + Home Assistant, while keeping the **OEM controller as an automatic failback**. A small signal relay switches the blue data wire between the OEM controller and the ESP32. If the ESP32 loses power or crashes, the relay de-energizes and the OEM controller reconnects automatically.

```
                         RELAY (SPDT)
                        ┌─────────────┐
  OEM Controller ───────┤ NC      COM ├──────── Blue Wire to Heater ECU
  (blue wire)           │             │
  ESP32 GPIO17 ──┐      │ NO          │
     (via level  ├──────┤             │
      shifter)   │      └──────┬──────┘
                 │             │ Coil
                 │      ┌──────┴──────┐
                 │      │  GPIO4 via  │
                 │      │  NPN driver │
                 │      └─────────────┘
                 │
  ESP32 GPIO16 ──┴──────────────────────────── Blue Wire (passive tap)
  (RX - always                                 (high impedance, always
   connected)                                   reads regardless of
                                                relay state)
```




### Operating Modes

| Relay State | GPIO4 | Mode | What Happens |
|-------------|-------|------|--------------|
| De-energized | LOW (default) | **Passive/OEM** | OEM controller talks to heater normally. ESP32 RX eavesdrops on both frames (48 bytes). All sensor entities update. Control switches are ignored. |
| Energized | HIGH | **ESP32 Control** | OEM controller disconnected. ESP32 TX sends command frames, RX reads heater responses. Full control via HA. |

---

## Pin Assignments

| ESP32 Pin | Function | Notes |
|-----------|----------|-------|
| **GPIO16** | UART RX (blue wire) | Your existing connection. Always connected to blue wire via passive tap. |
| **GPIO17** | UART TX (blue wire) | NEW. Connects to relay NO contact via level shifter circuit. Only active when relay is energized. |
| **GPIO4** | Relay control | NEW. Drives relay coil via NPN transistor. LOW = OEM (safe default). |

> **Why GPIO17?** On the ESP32-D, GPIO16 and GPIO17 are the default UART2 RX/TX pair, so they work well together. GPIO4 is a general-purpose pin that boots LOW, ensuring the relay stays de-energized during startup/reset.

---

## Circuit 1: TX Level Shifter (3.3V → 5V)

This converts the ESP32's 3.3V UART TX signal to the heater's 5V level using a single NPN transistor in open-drain configuration.

### Why not a commercial level shifter module?
You already found that the common BSS138-based bidirectional level shifters cause problems at 25,000 baud. Those modules add too much capacitance through their FETs and pull-ups, distorting the fast edges. The simple open-drain circuit below has far less capacitance and is the same approach the Afterburner project uses.

### Schematic

```
                                    +5V (from heater)
                                     │
                                     ├── 4.7kΩ pull-up
                                     │
 ESP32 GPIO17 ── 1kΩ ──┬── Base     │
                        │           ┌┴┐
                        │     NPN   │ │ ← Collector ──── To Relay NO
                        │  (2N2222  │ │                  (blue wire side)
                        │  or BC547)└┬┘
                        │           │
                       GND ←── Emitter
                                   │
                                  GND
```

### How it works
- **UART idle (TX HIGH, 3.3V):** Transistor turns ON → collector pulls LOW → BUT... this inverts the signal!

**Important: Signal inversion.** An NPN common-emitter stage inverts. UART idles HIGH. So we need to account for this. Two options:

#### Option A: Two-transistor non-inverting buffer (recommended)

```
                                         +5V (from heater)
                                          │
                                          ├── 4.7kΩ
                                          │
 ESP32 GPIO17 ── 1kΩ ──┬── B            ┌┴┐
                        │      Q1 NPN    │ │ C ──── To Relay NO
                        │    (2N2222)    │Q2│       (blue wire)
                       ┌┴┐              └┬┘ NPN
                   10kΩ│ │── C ── 1kΩ ──B│  (2N2222)
                       └┬┘               │
                        │                │
                       GND ── E ──────── E ── GND

  Q1 inverts: GPIO17 HIGH → Q1 on → Q1 collector LOW
  Q2 inverts again: Q1 collector LOW → Q2 off → pull-up holds wire HIGH ✓
  
  GPIO17 LOW → Q1 off → Q1 collector HIGH via 1kΩ → Q2 on → wire pulled LOW ✓
```

#### Option B: Single transistor with ESP-IDF UART TX inversion

The ESP32's UART peripheral can invert the TX signal in hardware. Add this to the ESPHome YAML inside an `on_boot` lambda, or we can set it in the component's `setup()`. This avoids needing the second transistor:

```
Single transistor circuit (use with TX inversion enabled):

                                    +5V (from heater)
                                     │
                                     ├── 4.7kΩ pull-up
                                     │
 ESP32 GPIO17 ── 1kΩ ──┬── Base     │
 (TX inverted           │    NPN    ┌┴┐
  in software)          │  (2N2222) │ │ Collector ── To Relay NO
                        │           └┬┘
                       GND      Emitter
                                   │
                                  GND

  With TX inversion: idle state = pin LOW → transistor OFF → pull-up HIGH ✓
  Start bit: pin HIGH → transistor ON → wire LOW ✓
```

To enable TX inversion, add to the component's setup() in cdh_controller.cpp:
```cpp
// In setup(), after UART is configured:
#include "driver/uart.h"
uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_TXD_INV);
```

**I recommend Option A (two transistors)** as it's more portable and doesn't require framework-specific code. Both transistors, two resistors, done.

### Parts for TX level shifter
| Part | Value | Notes |
|------|-------|-------|
| Q1, Q2 | 2N2222A or BC547 (NPN) | Any small-signal NPN works |
| R1 | 1kΩ | ESP32 to Q1 base |
| R2 | 10kΩ | Q1 emitter to GND (pull-down) |
| R3 | 1kΩ | Q1 collector to Q2 base |
| R4 | 4.7kΩ | Pull-up to 5V on output |

---

## Circuit 2: Relay Driver

A small signal relay controlled by GPIO4, with an NPN transistor driver and flyback diode.

```
                                    +5V or +3.3V (match relay coil voltage)
                                     │
                                    ┌┴┐
                                    │ │ Relay coil
                                    │ │ (SRD-05VDC or SRD-3.3VDC)
                                    └┬┘
                            Flyback  │
 ESP32 GPIO4 ── 1kΩ ──┬── B  diode ─┤─ (1N4148, cathode to +V)
                       │    NPN     ┌┴┐
                       │  (2N2222)  │ │ Collector
                       │            └┬┘
                      GND        Emitter
                                   │
                                  GND
```

### Parts for relay driver
| Part | Value | Notes |
|------|-------|-------|
| Relay | SRD-05VDC-SL-C or SRD-3.3VDC-SL-C | SPDT, signal-level. Must have NC + NO + COM. |
| Q3 | 2N2222A or BC547 | NPN transistor to drive relay coil |
| R5 | 1kΩ | GPIO4 to Q3 base |
| D1 | 1N4148 | Flyback protection across relay coil |

> **Alternative:** A pre-built relay module (the common 1-channel Arduino relay boards) already has the transistor driver and flyback diode built in. Just connect GPIO4 to the IN pin, and 5V/GND to the module. Much easier.

### Relay wiring
| Relay Terminal | Connects To |
|---------------|-------------|
| **COM** | Blue wire going TO THE HEATER ECU |
| **NC** (normally closed) | Blue wire FROM THE OEM CONTROLLER |
| **NO** (normally open) | Output of TX level shifter circuit |

---

## Circuit 3: RX Connection (unchanged)

Your existing passive tap stays the same:

```
  Blue wire ────────────── ESP32 GPIO16
  (from heater side,       (direct connection, no level shifter)
   after relay COM)
```

The ESP32 GPIO16 reads the 5V signal directly. While not officially 5V-tolerant, ESP32 GPIOs have internal clamping diodes that handle this safely at the low currents on a data line. This has been working reliably for you already.

**Optional protection:** If you want extra safety, add a simple resistor divider:
```
  Blue wire ── 10kΩ ──┬── ESP32 GPIO16
                      │
                     20kΩ
                      │
                     GND
```
This divides 5V down to ~3.3V. But given your current setup works fine without it, this is optional.

---

## Power Supply

The heater ECU provides **+5V** on the red wire of the controller cable. This can power the relay and level shifter circuit directly, and feed a 3.3V regulator for the ESP32.

```
  Heater +5V (red wire) ──┬── Relay coil (if 5V relay)
                           ├── Level shifter pull-up (4.7kΩ)
                           │
                           ├── AMS1117-3.3 or similar ── ESP32 3.3V input
                           │
  Heater GND (black wire) ─┴── Common ground
```

> **Note:** If your ESP32 dev board is USB-powered, it already has its own 3.3V regulator. Just make sure the **grounds are connected** between the ESP32, the heater, and the relay circuit. The 5V from the heater is only needed for the level shifter pull-up and relay.

---

## Complete BOM

| Qty | Part | Purpose | Approx Cost |
|-----|------|---------|-------------|
| 1 | ESP32-D dev board | You already have this | — |
| 3 | 2N2222A or BC547 NPN transistors | Q1 (level shift), Q2 (level shift), Q3 (relay driver) | $0.30 |
| 1 | 1N4148 diode | Flyback protection for relay | $0.05 |
| 1 | SRD-05VDC-SL-C relay (or 1-ch relay module) | Blue wire switching | $1.50 |
| 2 | 1kΩ resistors | Base drive for Q1, Q3 | $0.05 |
| 1 | 1kΩ resistor | Q1 collector to Q2 base | $0.02 |
| 1 | 10kΩ resistor | Q1 pull-down | $0.02 |
| 1 | 4.7kΩ resistor | Pull-up to 5V on blue wire | $0.02 |
| — | Wire, connectors | Hookup | — |
| | | **Total added cost** | **~$2** |

If using a **pre-built relay module** instead of discrete components for the relay driver, you can skip Q3, R5, and D1.

---

## Installation Steps

### 1. Prepare the component files
Copy the `cdh_controller/` folder into your ESPHome config directory:
```
~/.esphome/  (or wherever your ESPHome configs live)
├── greenhouse-diesel-heater.yaml
└── custom_components/
    └── cdh_controller/
        ├── __init__.py
        ├── cdh_controller.h
        ├── cdh_controller.cpp
        ├── sensor.py
        ├── text_sensor.py
        ├── binary_sensor.py
        ├── switch.py
        ├── number.py
        └── select.py
```

### 2. Build the circuits on a breadboard first
Test with the relay, level shifter, and existing heater before committing to permanent wiring.

### 3. Wire it up
1. Keep GPIO16 connected to the blue wire as it is now
2. Build the TX level shifter (two transistors + resistors)
3. Wire the relay: COM to heater blue wire, NC to OEM controller blue wire
4. Connect the level shifter output to relay NO terminal
5. Wire GPIO17 to the level shifter input
6. Wire GPIO4 to the relay driver
7. Connect grounds between ESP32, heater, and relay circuit

### 4. Flash and test in passive mode first
The component boots in **passive mode** by default (relay de-energized, OEM controller active). Verify all your sensors still work correctly before trying control mode.

### 5. Test control mode
1. In HA, flip the "ESP32 Control Mode" switch ON
2. The relay should click, disconnecting the OEM controller
3. Set temperature, pump Hz range, fan speed range via HA number entities
4. Flip "Heater Power" ON
5. Monitor the run state and sensor readings
6. Flip "ESP32 Control Mode" OFF to return to OEM control

### 6. Safety checks
- Verify that unplugging the ESP32 causes the relay to de-energize (OEM reconnects)
- Verify that the heater shuts down safely when control mode is disabled
- Check that error codes are reported correctly
- Test the comms timeout failback (if ESP32 loses contact for >5 seconds, it auto-reverts to OEM)

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| No sensor data in passive mode | Same as before - check GPIO16 connection | Verify blue wire tap |
| Relay clicks but no heater response | TX level shifter not working | Check with logic analyzer/scope at relay NO pin |
| Heater starts but immediately errors | Wrong command frame values | Check supply voltage setting matches your system (12V vs 24V) |
| E-05 error (comms timeout) | TX frames not reaching heater | Check relay wiring, level shifter output |
| Garbled sensor data in control mode | Half-duplex timing issue | May need to adjust RX_TIMEOUT_MS in code |
| Numbers still off (like pump Hz) | Same firmware calibration issue as before | Apply the correction factor in Node-RED as before |

---

## Notes on Your Pump Hz Issue

The pump Hz reading discrepancy (ESP reads lower than display) is a firmware-level calibration difference between your primary and backup heaters. When you switch to ESP32 control mode, the ESP32 sends the pump Hz range you specify, and the heater ECU controls the actual pump speed. The RX frame reports back what the heater is actually doing. The same linear correction (`Actual = ESP × 1.16 + 0.28`) should still apply to the sensor readings.

You could add a template sensor in HA to auto-correct this:
```yaml
# In HA configuration.yaml
template:
  - sensor:
      - name: "Corrected Pump Frequency"
        unit_of_measurement: "Hz"
        state: "{{ (states('sensor.greenhouse_diesel_heater_pump_frequency') | float * 1.16 + 0.28) | round(1) }}"
```
