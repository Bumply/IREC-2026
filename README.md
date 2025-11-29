# 🚀 ANKA - Zenith Rocket Team | IREC 2026

<p align="center">
  <img src="https://img.shields.io/badge/Competition-IREC%202026-red?style=for-the-badge" />
  <img src="https://img.shields.io/badge/Category-10K%20COTS-blue?style=for-the-badge" />
  <img src="https://img.shields.io/badge/Target-10,000%20ft%20AGL-green?style=for-the-badge" />
  <img src="https://img.shields.io/badge/University-Atılım%20University-orange?style=for-the-badge" />
</p>

<p align="center">
  <strong>From Ashes to Zenith: The Rebirth of Power</strong>
</p>

---

## 📋 Quick Stats

| Parameter | Value |
|-----------|-------|
| **Rocket Name** | ANKA |
| **Total Length** | 2.90 m |
| **Airframe Diameter** | 154 mm |
| **Liftoff Weight** | 27.51 kg |
| **Payload Mass** | 3.0 kg |
| **Motor** | AeroTech M2500T-PS |
| **Total Impulse** | 9,573 Ns |
| **Predicted Apogee** | 10,311 ft AGL |
| **Max Velocity** | 309 m/s (Mach 0.9) |

---

## 🏗️ Project Structure

```
irec/
├── docs/                    # Documentation & reports
│   ├── technical-report/    # IREC Technical Report
│   ├── presentations/       # Poster & podium materials
│   └── checklists/          # Pre-flight, arming, recovery checklists
├── avionics/                # Avionics subsystem
│   ├── flight-computer/     # SRAD flight computer
│   ├── schematics/          # PCB designs & schematics
│   ├── firmware/            # STM32 firmware code
│   └── ground-station/      # Ground station software
├── simulations/             # Flight simulations & analysis
│   ├── openrocket/          # OpenRocket files
│   ├── cfd/                 # ANSYS Fluent analysis
│   ├── structural/          # ANSYS Structural analysis
│   └── matlab/              # 6-DOF & trajectory analysis
├── recovery/                # Recovery system
│   ├── parachutes/          # Parachute designs & specs
│   └── deployment/          # Deployment mechanism
├── payload/                 # Payload subsystem
├── manufacturing/           # Manufacturing docs & drawings
└── tests/                   # Test reports & data
```

---

## 🎯 Subsystems Overview

### Avionics (Triple Redundancy)
| Component | Type | Function |
|-----------|------|----------|
| RRC3 Sport Altimeter | COTS | Primary dual-deployment |
| EasyMini Altimeter | COTS | Backup dual-deployment |
| SRAD Flight Computer | SRAD | Tertiary deployment + logging |
| Featherweight GPS | COTS | Position tracking (915 MHz) |
| LoRa E32-433T30D | SRAD | Telemetry (433 MHz) |

### SRAD Flight Computer Specs
- **MCU:** STM32F429ZIT6 (ARM Cortex-M4F @ 180MHz)
- **Altimeter:** MS5611 GY-63 (±10 cm, 100 Hz)
- **IMU:** MPU-9250 (±16g, 1 kHz) + BNO080/085
- **Storage:** W25Q40CLSNIG Flash (512 KB)
- **Pyro Channels:** 2x IRFU120 MOSFET drivers

### Recovery System
- **Drogue:** 0.85m diameter (deploys at apogee)
- **Main:** 3.0m diameter (deploys at 457m / 1,500 ft AGL)
- **Shock Cords:** 10m total (15 kN Perlon flat)

---

## 📅 Timeline

See [TODO.md](./TODO.md) for detailed task tracking.

---

## 👥 Team

**Zenith Rocket Team (ZRT)** - Atılım University, Ankara, Turkey

- 🌐 [Website](https://zenithrocketry.weebly.com)
- 📸 [Instagram](https://www.instagram.com/zenithrocket/)

---

## 📜 License

This project is for educational purposes as part of IREC 2026 competition.

