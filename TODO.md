# 📋 ANKA - IREC 2026 Master TODO

> **Last Updated:** November 29, 2025  
> **Competition Date:** June 2026 (Midland, Texas)

---

## 🔴 HIGH PRIORITY - Immediate Actions

### Administrative & Documentation
- [ ] **Submit Progress Report 1** - Check IMS for deadline
- [ ] **Obtain School Participation Letter** - Faculty signature required
- [ ] **Confirm Tripoli Insurance** - All 11 attending members need Lvl 0 membership
- [ ] **Book VISA appointments** - Start early (3 members already have US visa)
- [ ] **Finalize team roster** - 11 members attending

### Technical Report Preparation
- [ ] **Download latest AIAA template** from ESRA website
- [ ] **Assign report sections** to team members
- [ ] **Set internal deadlines** - Submit 3-5 days before ESRA deadline

---

## 🟡 AVIONICS SUBSYSTEM

### SRAD Flight Computer Development
- [ ] **Finalize PCB schematic** in Altium Designer
- [ ] **Component selection & BOM** - Order from JLCPCB
- [ ] **PCB layout** - 4-layer design
- [ ] **Order PCBs** - Allow 2-3 weeks shipping
- [ ] **Component assembly** - Team handles soldering
- [ ] **Firmware development** - STM32 code
  - [ ] MS5611 barometric driver
  - [ ] MPU-9250 IMU driver
  - [ ] BNO080/085 fusion driver
  - [ ] Flash memory logging
  - [ ] Pyro channel control logic
  - [ ] Apogee detection algorithm
  - [ ] Main deployment altitude trigger (500m AGL)
- [ ] **Ground testing** - Bench validation

### COTS Altimeters
- [ ] **RRC3 Sport configuration** - Dual deploy settings
- [ ] **EasyMini configuration** - Backup settings
- [ ] **Verify settings match** - Drogue @ apogee, Main @ 1,500 ft

### GPS & Telemetry
- [ ] **Featherweight GPS setup** - 915 MHz (no HAM required)
- [ ] **LoRa E32-433T30D integration** - 433 MHz telemetry
- [ ] **Ground station software** - Real-time data display
- [ ] **Range test** - Verify telemetry range

### Power System
- [ ] **Battery selection** - Li-Ion pairs for each system
- [ ] **Battery holder design** - Prevent spring compression under G-load
- [ ] **Schurter switches** - One per system
- [ ] **Wiring harness** - 22 AWG twisted pairs
- [ ] **Battery drain test** - Verify >2 hours operation

---

## 🟢 RECOVERY SUBSYSTEM

### Parachute Manufacturing
- [ ] **Drogue chute** - 0.85m, green/black, 8 panels
- [ ] **Main chute** - 3.0m, red/black, 12 panels
- [ ] **Payload chute** - 1.25m, 8 panels
- [ ] **Vent holes** - 1/10 diameter on each
- [ ] **Material:** 40D Ripstop Nylon

### Deployment Hardware
- [ ] **Hot gas generators (4x)** - Aluminum, primary + backup
- [ ] **Black powder charges** - 3g first sep, 3.4g second sep
- [ ] **E-match igniters** - Primary + redundant
- [ ] **Aluminum foil seals** - Pressure integrity

### Shock Cords & Hardware
- [ ] **Perlon flat cord** - 15 kN rated, 19mm width
- [ ] **Nose to upper body:** 4.5m
- [ ] **Upper to lower body:** 5.5m
- [ ] **M8 carabiners** - Locking type
- [ ] **M6 swivels** - Prevent line twist
- [ ] **Steel eyebolts** - All connection points
- [ ] **Fire-resistant fabric** - Parachute protection

---

## 🔵 STRUCTURES & MANUFACTURING

### Nose Cone
- [ ] **MDF mold fabrication**
- [ ] **Carbon fiber layup** - Von Kármán profile
- [ ] **Vacuum bagging & oven cure**
- [ ] **Aluminum 7075-T6 tip** - CNC machined
- [ ] **Final assembly & finish**

### Airframe
- [ ] **Upper body** - S-Glass fiberglass, 154mm diameter
- [ ] **Lower body** - S-Glass fiberglass
- [ ] **Aluminum tube mandrel** preparation
- [ ] **Vacuum bagging & curing**
- [ ] **Surface finish & paint**

### Fins
- [ ] **Carbon fiber layup** - Trapezoidal design
- [ ] **7.5mm thickness** - Flutter margin 63.8%
- [ ] **Glass surface mold**
- [ ] **Alignment jig** for mounting

### Internal Structure
- [ ] **Bulkheads** - Aluminum turning
- [ ] **Motor block** - Aluminum
- [ ] **Centering rings** - Aluminum
- [ ] **Avionics bay covers** - Upper/lower
- [ ] **Electronics sled** - 3D printed / PE1000
- [ ] **Coupling tubes**

---

## 🟣 PAYLOAD SUBSYSTEM

### 3U CubeSat Structure
- [ ] **100×100×300mm frame**
- [ ] **Mounting ring design**
- [ ] **Coupling tube interface**

### Sensors & Electronics
- [ ] **BME680 sensor** - Pressure, temp, humidity, VOC
- [ ] **GY-NEO6MV2 GPS module**
- [ ] **ESP32-CAM module** - Camera + SD card
- [ ] **SRAD sensor card** design
- [ ] **Custom ground station** integration

### Solar Panel System
- [ ] **4x solar panels** - 80×60mm, 1.5V/0.65W/430mA
- [ ] **Battery pack** - 3S2P Li-Ion
- [ ] **Passive BMS** design
- [ ] **NEMA-17 stepper motor** - Panel deployment
- [ ] **Switch system** - Altitude triggered (100m AGL)

---

## 🧪 TESTING SCHEDULE

### January 2026
- [ ] Shock cord pull test
- [ ] Eyebolt pull test
- [ ] Swivel pull test
- [ ] Montaged links pull test

### February 2026
- [ ] Electronic system tests
- [ ] Telemetry & GPS verification
- [ ] Altimeter configuration tests
- [ ] Battery drain tests

### March 2026
- [ ] Wind tunnel / CFD verification
- [ ] Parachute opening tests
- [ ] Black powder tests (1st deployment)
- [ ] Black powder tests (2nd deployment)
- [ ] Primary flight computer test
- [ ] Secondary flight computer test
- [ ] Logger flight computer test

### April 2026
- [ ] SRAD payload system test
- [ ] Full recovery system ground test
- [ ] Integrated system test
- [ ] **FULL SCALE TEST FLIGHT** (if possible)

---

## 📄 DELIVERABLES CHECKLIST

### Progress Updates (Check IMS for exact dates)
- [ ] Progress Report 1 - Entry form confirmation
- [ ] Progress Report 2 - Design update
- [ ] Progress Report 3 - Online video review with Safety Reviewers

### Technical Report
- [ ] Executive Summary (1 page)
- [ ] Introduction (2 pages)
- [ ] System Architecture Overview
  - [ ] Aero-structures
  - [ ] Propulsion
  - [ ] **Avionics**
  - [ ] Recovery
  - [ ] Telemetry
  - [ ] Payload
- [ ] Mission CONOPS
- [ ] Conclusions & Lessons Learned
- [ ] Acknowledgments
- [ ] **Appendix A:** System Weights & Performance Data
- [ ] **Appendix B:** Project Test Reports
- [ ] **Appendix C:** Hazard Analysis
- [ ] **Appendix D:** Risk Assessment
- [ ] **Appendix E:** Checklists
- [ ] **Appendix F:** Engineering Drawings

### Conference Materials
- [ ] **Poster** - 36"×48", self-supporting
- [ ] **Extended Abstract** - 500+ words, 2 pages max
- [ ] **Presentation slides** - PDF format

### Administrative
- [ ] School Participation Letter (faculty signed)
- [ ] Insurance documentation
- [ ] Team roster CSV

---

## ✈️ COMPETITION WEEK PREP

### Before Departure
- [ ] Print ALL checklists (hardcopy required!)
- [ ] Pack tools for altimeter data extraction
- [ ] Laptop + cables for post-flight inspection
- [ ] Spare batteries & components
- [ ] Team uniforms / matching shirts

### At Competition
- [ ] Registration & badge collection
- [ ] Conference day attendance (mandatory)
- [ ] Poster session setup
- [ ] Safety inspection preparation
- [ ] Launch preparation
- [ ] **Post-flight data inspection** - 45 min limit!
- [ ] Return rocket to DaVinci tent immediately

---

## 🏆 AWARDS WE'RE TARGETING

- [ ] **Category 1st/2nd/3rd Place** - 10K COTS
- [ ] **James Barrowman Award** - Flight dynamics (predicted vs actual apogee)
- [ ] **SDL Payload Challenge** - Functional payload

---

## 📝 Notes

### Lessons from IREC 2025
- Motor failure due to manufacturer defect (forward closure crack)
- Upper body, nose, avionics recovered intact
- Design & documentation received positive feedback
- **Action:** Thoroughly inspect all COTS motor components before flight

### Key Rules to Remember
- Late submissions = **DISQUALIFICATION** (no grace period)
- 3 safety/conduct violations = **DISQUALIFICATION**
- Apogee outside ±30% of target = **ZERO flight score**
- Must report to Post-Flight Inspection immediately after recovery
- Bring printed checklists or no flight!

---

> *"From Ashes to Zenith"* 🔥🚀

