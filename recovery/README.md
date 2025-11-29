# Recovery Subsystem - ANKA IREC 2026

## Overview

Dual-deployment recovery system with triple-redundant activation.

## System Architecture

```
         APOGEE (10,311 ft)
              │
              ▼
    ┌─────────────────────┐
    │   1ST SEPARATION    │
    │   (Drogue Deploy)   │
    │                     │
    │  Black Powder: 3g   │
    │  Hot Gas Gen: 2x    │
    └─────────────────────┘
              │
              │  Drogue Chute: 0.85m
              │  Descent Rate: ~25 m/s
              │
              ▼
    ┌─────────────────────┐
    │   2ND SEPARATION    │
    │    @ 1,500 ft AGL   │
    │      (457m)         │
    │                     │
    │  Black Powder: 3.4g │
    │  Hot Gas Gen: 2x    │
    └─────────────────────┘
              │
              │  Main Chute: 3.0m
              │  Descent Rate: <11 m/s (36 ft/s)
              │
              ▼
    ┌─────────────────────┐
    │    TOUCHDOWN        │
    │    Safe Landing     │
    └─────────────────────┘
```

## Parachutes

| Parachute | Diameter | Panels | Vent Hole | Color | Purpose |
|-----------|----------|--------|-----------|-------|---------|
| Drogue | 0.85m | 8 | 85mm | Green/Black | Initial stabilization |
| Main | 3.0m | 12 | 300mm | Red/Black | Final descent |
| Payload | 1.25m | 8 | 125mm | Neon | CubeSat recovery |

### Specifications
- **Material:** 40D Ripstop Nylon
- **Drag Coefficient:** Cd = 0.8
- **Vent Hole:** 1/10 of diameter (stability)
- **Cord Length:** 1.75× inflated diameter

## Shock Cords

| Section | Length | Location |
|---------|--------|----------|
| Upper | 4.5m | Nose ↔ Upper Body |
| Lower | 5.5m | Upper Body ↔ Lower Body |
| **Total** | **10m** | ~3× rocket length |

### Specifications
- **Type:** Perlon Flat
- **Width:** 19mm
- **Strength:** 15,000 N (15 kN)
- **Protection:** Fire-resistant fabric wrap

## Hardware

| Component | Specification | Quantity |
|-----------|--------------|----------|
| M8 Carabiners | Locking, cast steel | 5 |
| M6 Swivels | 560 kg tensile | 3 |
| Steel Eyebolts | M8 thread | 5 |

## Hot Gas Generators

```
┌───────────────────────────────────────┐
│         HOT GAS GENERATOR             │
├───────────────────────────────────────┤
│                                       │
│   ┌─────────────────────────────┐     │
│   │    Aluminum Tube Housing    │     │
│   │                             │     │
│   │  ┌───────────────────────┐  │     │
│   │  │   Black Powder Charge │  │     │
│   │  │   (sealed with Al     │  │     │
│   │  │    foil for pressure) │  │     │
│   │  └───────────────────────┘  │     │
│   │                             │     │
│   │  ┌─────────┐  ┌─────────┐   │     │
│   │  │Primary  │  │ Backup  │   │     │
│   │  │E-Match  │  │ E-Match │   │     │
│   │  └─────────┘  └─────────┘   │     │
│   │                             │     │
│   └─────────────────────────────┘     │
│                                       │
│   Configuration: 2 per separation     │
│   Total: 4 hot gas generators         │
│                                       │
└───────────────────────────────────────┘
```

### Black Powder Charges
| Separation | Primary | Backup | Total per event |
|------------|---------|--------|-----------------|
| 1st (Drogue) | 3.0g | 3.0g | 6.0g |
| 2nd (Main) | 3.4g | 3.4g | 6.8g |

## Deployment Redundancy

```
        ┌──────────────┐
        │    RRC3      │───┐
        │  (PRIMARY)   │   │
        └──────────────┘   │
                           │    ┌────────────────┐
        ┌──────────────┐   ├───►│  HOT GAS GEN   │
        │   EasyMini   │───┤    │   PRIMARY      │
        │   (BACKUP)   │   │    └────────────────┘
        └──────────────┘   │
                           │    ┌────────────────┐
        ┌──────────────┐   ├───►│  HOT GAS GEN   │
        │    SRAD      │───┘    │    BACKUP      │
        │  (TERTIARY)  │        └────────────────┘
        └──────────────┘
```

**Triple redundancy:** If ANY one of the three flight computers fires, both hot gas generators have primary AND backup igniters.

## Descent Calculations

### Drogue Phase (Apogee → 457m)
- **Altitude Drop:** ~2,690m
- **Estimated Time:** ~108 seconds
- **Descent Rate:** ~25 m/s

### Main Phase (457m → Ground)
- **Altitude Drop:** 457m
- **Estimated Time:** ~42 seconds
- **Descent Rate:** <11 m/s (requirement)

### Total Descent Time: ~150 seconds (2.5 minutes)

## Testing Schedule

### January 2026
- [ ] Shock cord pull test (15 kN load)
- [ ] Eyebolt pull test
- [ ] Swivel pull test
- [ ] Montaged links assembly test

### March 2026
- [ ] Parachute opening test (ground deployment)
- [ ] Black powder charge test - 1st separation (3g)
- [ ] Black powder charge test - 2nd separation (3.4g)
- [ ] Full ground deployment test

### April 2026
- [ ] Integrated system test
- [ ] Flight test (if possible)

## Safety Notes

⚠️ **CRITICAL REMINDERS:**
- Always use fire-resistant fabric to protect parachutes
- Verify e-match continuity before arming
- Follow printed checklists at all times
- Pressure vent in avionics bay required for accurate altimeter readings

