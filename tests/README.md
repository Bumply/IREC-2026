# Tests - ANKA IREC 2026

## Test Schedule Overview

| Month | Focus Area |
|-------|------------|
| January 2026 | Structural tests (shock cords, hardware) |
| February 2026 | Electronics & avionics tests |
| March 2026 | Recovery system tests, CFD validation |
| April 2026 | Integrated system tests, full-scale flight |

---

## January 2026 - Structural Tests

### Shock Cord Pull Test
- **Objective:** Verify 15 kN load rating
- **Equipment:** Tensile testing machine
- **Pass Criteria:** No failure at 15 kN
- **Status:** [ ] Planned

### Eyebolt Pull Test
- **Objective:** Verify structural integrity at attachment points
- **Equipment:** Tensile testing machine
- **Pass Criteria:** SF > 2.0
- **Status:** [ ] Planned

### Swivel Pull Test
- **Objective:** Verify 560 kg rating under rotation
- **Equipment:** Tensile testing machine
- **Pass Criteria:** No failure at rated load
- **Status:** [ ] Planned

### Montaged Links Pull Test
- **Objective:** Verify assembled connection strength
- **Equipment:** Tensile testing machine
- **Pass Criteria:** Weakest link > 5 kN
- **Status:** [ ] Planned

---

## February 2026 - Electronics Tests

### Battery Drain Test
- **Objective:** Verify >2 hours operation
- **Procedure:** Power all systems, log voltage over time
- **Pass Criteria:** All systems operational at T+2 hours
- **Status:** [ ] Planned

### Telemetry Range Test
- **Objective:** Verify LoRa range
- **Procedure:** Transmit at increasing distances
- **Pass Criteria:** Clear signal at 3+ km
- **Status:** [ ] Planned

### GPS Accuracy Test
- **Objective:** Verify position accuracy
- **Procedure:** Compare GPS vs known coordinates
- **Pass Criteria:** <5m CEP
- **Status:** [ ] Planned

### Altimeter Configuration Test
- **Objective:** Verify deployment altitudes
- **Procedure:** Pressure chamber simulation
- **Pass Criteria:** Correct trigger at apogee & 457m
- **Status:** [ ] Planned

---

## March 2026 - Recovery Tests

### Parachute Opening Test
- **Objective:** Verify full deployment
- **Procedure:** Ground deployment test
- **Pass Criteria:** Full inflation, no tangles
- **Status:** [ ] Planned

### Black Powder Test - 1st Separation
- **Objective:** Verify 3g charge separation
- **Procedure:** Ground test with inert body
- **Pass Criteria:** Clean separation
- **Status:** [ ] Planned

### Black Powder Test - 2nd Separation
- **Objective:** Verify 3.4g charge separation
- **Procedure:** Ground test with inert body
- **Pass Criteria:** Clean separation
- **Status:** [ ] Planned

### Primary Flight Computer Test
- **Objective:** Verify RRC3 e-match firing
- **Procedure:** Simulated flight profile
- **Pass Criteria:** Fires at correct altitudes
- **Status:** [ ] Planned

### Secondary Flight Computer Test
- **Objective:** Verify EasyMini backup
- **Procedure:** Simulated flight profile
- **Pass Criteria:** Fires at correct altitudes
- **Status:** [ ] Planned

### SRAD Flight Computer Test
- **Objective:** Verify logging and pyro
- **Procedure:** Simulated flight profile
- **Pass Criteria:** Data logged, fires at correct altitudes
- **Status:** [ ] Planned

---

## April 2026 - Integration Tests

### SRAD Payload System Test
- **Objective:** Verify sensor telemetry
- **Procedure:** Full mission simulation
- **Pass Criteria:** GPS, P/T/H/VOC data received at ground station
- **Status:** [ ] Planned

### Full Recovery Ground Test
- **Objective:** End-to-end deployment
- **Procedure:** Simulated apogee + main deployment
- **Pass Criteria:** Both chutes deploy correctly
- **Status:** [ ] Planned

### Full-Scale Test Flight
- **Objective:** Validate entire system
- **Location:** TBD (local launch site)
- **Pass Criteria:** Nominal flight & recovery
- **Status:** [ ] Planned (if time permits)

---

## Test Report Template

```markdown
# Test Report: [Test Name]

**Date:** YYYY-MM-DD
**Location:** 
**Conducted By:**

## Objective


## Equipment Used


## Procedure


## Results


## Pass/Fail


## Notes & Observations


## Photos/Videos

```

---

## DTEG Required Tests

Per IREC Design, Test & Evaluation Guide:

- [ ] Recovery System [DTEG 6.13]
- [ ] Ground Test Demonstration [DTEG 6.13.4]
- [ ] Flight Test Demonstration [DTEG 6.13.5]
- [ ] Dual Redundancy of Recovery Electronics [R&R 2.6.2.10.a]
- [ ] Stored-Energy Devices – Energetic Device Safing and Arming [DTEG 6.14]
- [ ] Arming Devices [DTEG 6.15]
- [ ] Arming Device Verification [DTEG 6.16]

