# Payload - ANKA IREC 2026

## Mission: Habitability Sensor Platform

3U CubeSat payload demonstrating sustainable power and environmental sensing.

## Form Factor

| Dimension | Value |
|-----------|-------|
| Size | 3U (100×100×300 mm) |
| Mass | 3.0 kg |
| Format | ESRA Payload Cube Unit |

**✅ Qualifies for 50 bonus points** (CubeSat form factor with functional payload)

## Subsystems

### Solar Power System
- **4× Solar Panels:** 80×60 mm, 1.5V / 0.65W / 430mA each
- **Battery Pack:** 6× Li-Ion cells (3S2P configuration)
- **BMS:** Passive balancing with overcharge protection
- **Deployment:** NEMA-17 stepper motor (200 step, 1.3A)
- **Trigger:** Altitude-based (100m AGL)

### Sensors
| Sensor | Measurements |
|--------|--------------|
| **BME680** | Pressure, Temperature, Humidity, VOC |
| **GY-NEO6MV2** | GPS position tracking |
| **ESP32-CAM** | OV2640 camera + SD card logging |

### Communication
- **LoRa E32-433T30D:** 433 MHz telemetry to ground station
- **Data Rate:** Real-time sensor streaming

## Block Diagram

```
┌─────────────────────────────────────────────────────┐
│                  3U CUBESAT PAYLOAD                  │
├─────────────────────────────────────────────────────┤
│                                                     │
│  ┌─────────────┐         ┌─────────────────────┐   │
│  │ Solar Panel │──┐      │    SRAD Sensor      │   │
│  │    Array    │  │      │       Card          │   │
│  │   (4 × 80   │  │      │                     │   │
│  │    ×60mm)   │  │      │  ┌───────────────┐  │   │
│  └─────────────┘  │      │  │    BME680     │  │   │
│                   │      │  │ P/T/H/VOC     │  │   │
│  ┌─────────────┐  │      │  └───────────────┘  │   │
│  │   Battery   │◄─┘      │                     │   │
│  │   Pack      │         │  ┌───────────────┐  │   │
│  │  (3S2P)     │────────►│  │  NEO6MV2 GPS  │  │   │
│  │             │         │  └───────────────┘  │   │
│  │ + Passive   │         │                     │   │
│  │    BMS      │         │  ┌───────────────┐  │   │
│  └─────────────┘         │  │   ESP32-CAM   │  │   │
│                          │  │  + SD Card    │  │   │
│  ┌─────────────┐         │  └───────────────┘  │   │
│  │  NEMA-17    │         │                     │   │
│  │  Stepper    │         │  ┌───────────────┐  │   │
│  │  (Deploy)   │         │  │ LoRa 433MHz   │  │   │
│  └─────────────┘         │  │  Telemetry    │  │   │
│                          │  └───────────────┘  │   │
│                          └─────────────────────┘   │
│                                                     │
└─────────────────────────────────────────────────────┘
```

## SDL Payload Challenge

This payload is designed for entry into the **Space Dynamics Laboratory (SDL) Payload Challenge**.

### Evaluation Criteria
- Scientific merit
- Innovation
- Build quality
- Data quality
- Mission success

## Testing

- [ ] Solar panel power output verification
- [ ] Battery charge/discharge cycling
- [ ] BMS overcharge protection test
- [ ] Stepper motor deployment test
- [ ] Sensor calibration
- [ ] Telemetry range test
- [ ] Camera/SD card logging test
- [ ] Full integrated system test

