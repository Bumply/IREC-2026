# Simulations - ANKA IREC 2026

## Tools Used

| Tool | Purpose | Files |
|------|---------|-------|
| **OpenRocket** | Flight simulation, stability | `*.ork` |
| **ANSYS Fluent** | CFD analysis | `/cfd/` |
| **ANSYS Structural** | Structural analysis | `/structural/` |
| **MATLAB Simulink** | 6-DOF, trajectory | `/matlab/` |

## Key Parameters

### Flight Profile
| Parameter | Value |
|-----------|-------|
| Target Apogee | 10,000 ft AGL |
| Predicted Apogee | 10,311 ft AGL |
| Max Velocity | 309 m/s (Mach ~0.9) |
| Max Acceleration | 10.05 G |
| Rail Departure Velocity | 33.2 m/s |
| Thrust-to-Weight | 9.12:1 |

### Stability
| Parameter | Value |
|-----------|-------|
| Min Static Margin | 1.57 cal |
| Max Static Margin | 4.26 cal |
| Static @ Mach 0.3 | 2.94 cal |

### Fin Flutter
| Parameter | Value |
|-----------|-------|
| Flutter Velocity | 506.14 m/s |
| Safety Margin | 63.8% |
| Method | NACA TN 4197 |

## Analysis Checklist

- [ ] OpenRocket baseline simulation
- [ ] CFD analysis - nose cone
- [ ] CFD analysis - fins
- [ ] CFD analysis - full body
- [ ] Structural analysis - nose cone
- [ ] Structural analysis - fins
- [ ] Structural analysis - airframe
- [ ] Motor retention analysis
- [ ] Centering ring stress analysis
- [ ] 6-DOF trajectory in MATLAB
- [ ] Barrowman CP validation (Excel)
- [ ] Monte Carlo wind analysis (0-20 m/s)

## Validation Plan

1. Compare OpenRocket CP with Barrowman equations
2. Verify CFD drag matches OpenRocket predictions
3. Confirm structural safety factors >1.5
4. Test flight correlation with predictions

