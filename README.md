# CaneVision_Smart_White_Cane_for_the_Visually_Impaired
This is the repository of my individual project of a assitive project for the visually impaired in Pioneer Academics with Carlotta Berry Ph.D. 
<div align="center">

#  CaneVision: Attachable Navigation Module for White Cane
### A Low-Cost, Multi-Sensor Embedded Spatial Perception & Adaptive Feedback System

[![Platform: Arduino Uno R3](https://img.shields.io/badge/Platform-Arduino%20Uno%20R3-00979D.svg?logo=arduino)](https://www.arduino.cc/)
[![Language: C/C++](https://img.shields.io/badge/Language-Embedded%20C%2FC%2B%2B-00599C.svg)](https://isocpp.org/)
[![Cost: $41.94](https://img.shields.io/badge/BOM%20Cost-%2441.94-success.svg)]()
[![Demo: YouTube](https://img.shields.io/badge/Live%20Demo-YouTube%20Shorts-FF0000.svg?logo=youtube)](https://youtube.com/shorts/EYo3PqKyFFs?feature=share)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

**Author:** Jingkun Li (Jarred)  
**Advisor:** Prof. Carlotta Berry, Ph.D.  
*Pioneer Academics Research Program — Introduction to Electronics, Microcontrollers, and Robotics (Summer 2026)*

[ **Watch Live Demo on YouTube**](https://youtube.com/shorts/EYo3PqKyFFs?feature=share) | [ **Read Full Research Paper**](docs/Jingkun_Li_Jarred_CaneVision_Research_Final_Paper.pdf)

</div>

---

##  Executive Summary & Motivation

Around the world, approximately 2.2 billion people live with vision impairment. While the traditional white cane remains the most widely adopted mobility aid due to its simplicity and affordability, it suffers from severe physical blind spots—specifically **upper-body/head-level overhanging hazards (0.8 m – 2.0 m)** and **sudden ground drop-offs** prior to physical tip contact. Conversely, commercial Electronic Travel Aids (ETAs) cost between $2,500 and $4,900+ and often induce auditory sensory overload.

**CaneVision** is a lightweight (<150 g), ultra-affordable (**$41.94 total BOM cost**), attachable smart navigation module designed to upgrade standard white canes (13–16 mm shaft diameter). Powered by an **Arduino Uno R3**, the system integrates multi-directional sensor fusion with advanced embedded signal processing algorithms to deliver intuitive, non-intrusive spatial awareness.

---

##  Key Engineering & Algorithmic Innovations

Rather than relying on naive blocking loops (`delay()`) or static thresholds, **CaneVision (V3.0)** introduces four core software and systems engineering innovations:

### 1. Non-Blocking Concurrency Architecture (`millis()`)
Eliminates single-core blocking delays by implementing an asynchronous timestamp-driven polling architecture (`loopInterval = 50 ms`) alongside an independent high-frequency acoustic timing loop. This guarantees concurrent multi-sensor sampling and keeps total system latency strictly **< 100 ms** (providing a ~1.5 s reaction buffer at a standard pedestrian walking speed of $1.2\text{ m/s}$).

### 2. Moving Average Filter for Ultrasonic Ranging ($N = 5$)
To suppress transient acoustic interference and specular reflection spikes in the HC-SR04 sensor, a circular buffer moving average filter is implemented:

$$D_{\text{average}} = \frac{1}{N} \sum_{i=1}^{N} D_i \quad (N = 5)$$

* **Mathematical Justification:** With a $50\text{ ms}$ polling interval, $N=5$ introduces a bounded verification window of $250\text{ ms}$. At an average walking speed of $1.2\text{ m/s}$, the user travels at most $30\text{ cm}$ during this window—safely within the $50\text{ cm}$ critical Danger Zone buffer. Also includes a cold-start pre-population routine in `setup()` to eliminate initial startup false alarms.

### 3. Environmental Adaptive Learning via Leaky Integrator ($\alpha = 0.99$)
Downward-facing Infrared (IR) drop-off sensors suffer from high false-positive rates when transitioning across surfaces with varying reflectivity (e.g., white tile to dark carpet). CaneVision implements a dynamic baseline calibration using a first-order **Leaky Integrator**:

$$B_{\text{new}} = \alpha \cdot B_{\text{old}} + (1 - \alpha) \cdot V_{\text{current}} \quad (\alpha = 0.99)$$

* **Time Constant Derivation:** Setting $\alpha = 0.99$ at $T_s = 50\text{ ms}$ yields a time constant $\tau \approx \frac{T_s}{1 - \alpha} = 5\text{ seconds}$. This filters out transient sweeping perturbations ($<1\text{ s}$) while smoothly adapting the drop-off threshold (`irBaseline + 150`) to new floor materials over ~5 seconds.

### 4. Dual Progressive Feedback System (Sensory Preservation)
To prevent cognitive overload and preserve the blind user's natural auditory awareness of traffic, CaneVision prioritizes **proportional PWM haptic feedback** and restricts continuous audio alarms to critical emergencies:
* **Safe Zone ($d > 150\text{ cm}$):** Haptic OFF (`PWM = 0`), Buzzer OFF.
* **Warning Zone ($50\text{ cm} < d \le 150\text{ cm}$):** Linearly mapped PWM vibration (`PWM: 80 -> 255`) paired with distance-proportional pulsing beeps (`Interval: 800 ms -> 100 ms`).
* **Danger Zone ($d \le 50\text{ cm}$ or Drop-Off Detected):** Immediate override triggering maximum continuous vibration (`PWM = 255`, $5.00\text{ V}$) and solid acoustic alarm.

---

## Hardware Architecture, Pin Mapping & BOM

### 1. Pinout Configuration
| Component | Arduino Uno Pin | Signal Type | Functional Description |
| :--- | :---: | :---: | :--- |
| **Ultrasonic Sensor (HC-SR04)** | `D9` (Trig) / `D10` (Echo) | Digital Out / In | Forward mid-air obstacle ranging ($20\text{ cm} - 200\text{ cm}$) |
| **Infrared (IR) Sensor** | `A0` | Analog Input | Downward surface reflection & sudden drop-off detection |
| **Photoresistor (LDR)** | `A1` | Analog Input | Ambient illumination monitoring (`DARK_THRES = 300`) |
| **Vibrating Mini Motor Disc** | `D3` | PWM Output | Proportional tactile feedback (`analogWrite: 0 - 255`) |
| **Active Piezo Buzzer** | `D5` | Digital Output | Dynamic interval pulsing & emergency acoustic alert |
| **Safety Warning LED** | `D6` | Digital Output | Automatic external visibility beacon for pedestrians at night |

### 2. Bill of Materials (BOM — Total Under $50 Constraint)
| Part Description | Qty | Unit Cost | Subsystem Role |
| :--- | :---: | :---: | :--- |
| Arduino Uno R3 | 1 | $13.50 | Central Microcontroller (5V Logic, ~50mA draw) |
| Sharp / Analog IR Sensor | 1 | $14.95 | Ground Drop-off Detection |
| Breadboard & Wiring | 1 | $4.95 | Prototyping Interconnect |
| HC-SR04 Ultrasonic Sensor | 1 | $3.95 | Upper-Body / Forward Obstacle Detection |
| Vibrating Mini Motor Disc | 1 | $1.95 | Haptic Feedback Actuator |
| Piezo Buzzer | 1 | $1.50 | Auditory Alert Actuator |
| Photoresistor (LDR) | 1 | $0.95 | Ambient Light Sensing |
| High-Brightness LED | 1 | $0.19 | Nighttime Pedestrian Alert |
| **Total Prototype Cost** | **8** | **$41.94** | **Meets < $50 Economic Design Constraint** |

---

##  Empirical Verification & Validation Results

### 1. Infrared Dynamic Baseline & Drop-off Validation
| Test Surface Condition | Raw IR Analog Value | Dynamic Baseline ($B$) | System Response |
| :--- | :---: | :---: | :--- |
| Flat White Surface (Boot Calibration) | `52` | `52` | Normal Navigation |
| Flat Dark Surface (Material Shift) | `105` | `105` *(Adapted over 5s)* | Smooth Adaptation (No False Alarm) |
| Table Edge (Transitional) | `280` | `105` *(Held by filter)* | **Drop-Off Hazard Triggered** |
| Mid-Air / Stairs Drop-Off | `640` | `105` *(Held by filter)* | **Immediate Max Override Alarm** |

### 2. Proportional Haptic Feedback (PWM Multimeter Verification)
| Simulated Distance ($d$) | Operational Zone | PWM Command (`0-255`) | Measured Terminal Voltage | Perceived Tactile Intensity |
| :---: | :---: | :---: | :---: | :--- |
| $> 150\text{ cm}$ | Safe Zone | `0` | $0.00\text{ V}$ | OFF (No Vibration) |
| $120\text{ cm}$ | Warning Zone | `132` | $2.60\text{ V}$ | Weak, noticeable pulsing |
| $80\text{ cm}$ | Warning Zone | `202` | $3.96\text{ V}$ | Moderate, continuous vibration |
| $\le 50\text{ cm}$ | Danger Zone | `255` | $5.00\text{ V}$ | Maximum intensity emergency alert |

---

##  Repository Structure

```text
CaneVision/
├── firmware/
│   └── CaneVision_V3/
│       └── CaneVision_V3.ino      # Complete Arduino C++ source code (V3.0)
├── docs/
│   └── CaneVision_Paper.pdf       # Full research paper & engineering appendices
├── assets/
│   ├── block_diagram.png          # System operational workflow (Figure 1)
│   ├── circuit_schematic.png      # Tinkercad circuit layout (Appendix C)
│   └── hardware_prototype.jpg     # Final assembled module on white cane (Appendix D)
└── README.md                      # Project documentation
