# RP2350-Touch-LCD-2.8 Motion Tracking System
## Design and Implementation Plan

**Version:** 1.2  
**Date:** 02/04/2026  
**Hardware Platform:** Waveshare RP2350-Touch-LCD-2.8

---

## 1. Hardware Overview

### 1.1 Core Components

| Component | Chip | Interface | Key Specifications |
|-----------|------|-----------|-------------------|
| Microcontroller | RP2350 | - | Dual-core ARM Cortex-M33 / RISC-V, 150MHz, 520KB SRAM, 16MB Flash |
| Display | ST7789T3 | SPI | 2.8" IPS LCD, 240×320 resolution, 262K colours |
| Touch Panel | CST328 | I2C | 5-point capacitive touch with interrupt support |
| IMU | QMI8658 | I2C | 6-axis (3-axis accelerometer + 3-axis gyroscope) |
| RTC | PCF85063 | I2C | Real-time clock for timestamps |
| Storage | TF Card Slot | SPI | MicroSD for map data and logging |
| Power | Li-ion Charger | - | Battery management for portable operation |
| Audio | Speaker | PWM | Audio feedback capability |

### 1.2 IMU Specifications (QMI8658)

**Accelerometer:**
- Resolution: 16-bit
- Selectable ranges: ±2g, ±4g, ±8g, ±16g
- Recommended setting: ±8g for vehicles, ±16g for aircraft

**Gyroscope:**
- Resolution: 16-bit
- Selectable ranges: ±16 to ±2048 °/sec
- Recommended setting: ±256°/sec for vehicles, ±512°/sec or higher for aircraft

### 1.3 Hardware Limitations

- No magnetometer (heading drift will occur without external reference)
- No barometric pressure sensor (altitude estimation limited to acceleration integration)
- IMU maximum range of ±16g may clip during aggressive aircraft manoeuvres
- No direct vehicle bus interface (OBD-II would require external adapter)

---

## 2. Assumptions

This section documents the key assumptions underlying the system design. These assumptions inform sensor configuration choices, algorithm design, and expected use cases.

### 2.1 Human Occupancy Assumption

**Primary Assumption:** The device will always be carried by or located with a human operator. Therefore, any vehicle or aircraft in which the device is used will not be performing high-G manoeuvres that would be unsafe or uncomfortable for human occupants.

**Implications:**

| Factor | Assumed Limit | Rationale |
|--------|---------------|-----------|
| Sustained acceleration | ≤2g | Human comfort limit for extended periods |
| Peak acceleration | ≤4g | Brief peaks tolerable in normal transport |
| Maximum acceleration | ≤6g | Extreme but survivable (roller coasters, emergency manoeuvres) |
| Sustained turn rate | ≤45°/sec | Comfortable vehicle/aircraft turns |
| Peak turn rate | ≤180°/sec | Aggressive but controlled manoeuvres |

**Design Consequence:** The ±8g accelerometer setting provides adequate headroom for all human-occupied scenarios. The ±16g setting is available but expected to be rarely needed outside of turbulence or emergency situations.

### 2.2 Environmental Assumptions

| Assumption | Value/Condition | Notes |
|------------|-----------------|-------|
| Operating temperature | -10°C to +50°C | Typical human-occupied environments |
| Vibration environment | Moderate | Road vehicles, light aircraft, commercial aviation |
| Altitude range | -500m to +12,000m MSL | Sea level to commercial aircraft cruise |
| Humidity | 10% to 90% RH | Non-condensing |

### 2.3 Use Case Assumptions

**Pedestrian Mode:**
- User is walking or running (not cycling, skating, etc.)
- Device is carried in hand, pocket, or bag
- Typical stride length 0.5m to 1.2m
- Movement speed 0 to 25 km/h

**Vehicle Mode:**
- Standard road vehicles (cars, vans, buses, trucks)
- Normal driving conditions (no racing, off-road, or stunt driving)
- Speed range 0 to 200 km/h
- Device mounted or placed stably in vehicle

**Aircraft Mode:**
- General aviation, light aircraft, commercial aviation, gliders
- Normal flight operations (no aerobatics, combat manoeuvres)
- Speed range 0 to 500 knots (ground speed)
- Altitude changes at normal climb/descent rates (≤2,000 ft/min typical)

### 2.4 Operational Assumptions

| Assumption | Description |
|------------|-------------|
| User interaction | User can view display and interact with touch during operation |
| Starting position | User can manually set starting position on map, or will use GPS in future |
| Calibration | User can keep device stationary for 5-10 seconds during calibration |
| Power availability | Li-ion battery provides minimum 2-4 hours operation |
| SD card | FAT32 formatted microSD card present for map data and logging |

### 2.5 Accuracy Expectations

Given the hardware limitations and human-occupied use cases, the following accuracy expectations are realistic:

| Metric | Pedestrian | Vehicle | Aircraft | Notes |
|--------|------------|---------|----------|-------|
| Distance accuracy | ±5% | ±10-20% | ±15-25% | Without GPS correction |
| Speed accuracy | ±1 km/h | ±5 km/h | ±10 knots | Averaged over 10+ seconds |
| Heading accuracy | ±15° | ±20° | ±25° | Drifts without magnetometer |
| Position drift | 1-2m/min | 5-10m/min | 10-20m/min | Cumulative error |

**Note:** These figures assume no GPS input. With GPS integration, position accuracy improves to GPS receiver specification (typically 2-5m CEP).

### 2.6 Exclusions

The following use cases are explicitly **not supported** by this design:

- Unmanned vehicles (drones, autonomous vehicles)
- Racing vehicles or high-performance driving
- Aerobatic aircraft or military aviation
- Spacecraft or suborbital vehicles
- Underwater vehicles
- High-vibration industrial equipment
- Use without human supervision

### 2.7 Future Assumption Reviews

These assumptions should be reviewed if:
- GPS module is integrated (accuracy expectations will change)
- Magnetometer is added (heading accuracy will improve)
- Barometer is added (altitude accuracy will improve)
- New use cases are identified that fall outside current assumptions

---

## 3. Units and Measurements Standard

### 3.1 Primary Unit System

This project uses the **International System of Units (SI)** as the primary standard, with user-selectable display formats.

### 3.2 Distance Units

| Unit | Symbol | Definition | Usage Context |
|------|--------|------------|---------------|
| Metre | m | SI base unit | Internal calculations, short distances |
| Kilometre | km | 1,000 m | Road/ground distances |
| Nautical Mile | NM | 1,852 m | Aircraft mode distances |
| Foot | ft | 0.3048 m | Aircraft altitude (optional display) |

**Internal Storage:** All distances stored in metres (m) as floating-point values.

### 3.3 Speed Units

| Unit | Symbol | Conversion | Usage Context |
|------|--------|------------|---------------|
| Metres per second | m/s | SI derived unit | Internal calculations |
| Kilometres per hour | km/h | 3.6 × m/s | Vehicle mode display (default) |
| Miles per hour | mph | 2.237 × m/s | Vehicle mode display (imperial) |
| Knots | kn | 1.944 × m/s | Aircraft mode display |
| Feet per minute | ft/min | 196.85 × m/s | Vertical speed (aircraft) |

**Internal Storage:** All speeds stored in metres per second (m/s).

### 3.4 Acceleration Units

| Unit | Symbol | Definition | Usage Context |
|------|--------|------------|---------------|
| Metres per second squared | m/s² | SI derived unit | Internal calculations |
| Standard gravity | g | 9.80665 m/s² | Sensor configuration, display |

**Internal Storage:** Accelerometer data stored in m/s², converted from raw sensor values.

**Note:** Per Section 2.1, expected acceleration range is ≤6g for human-occupied scenarios.

### 3.5 Angular Units

| Unit | Symbol | Definition | Usage Context |
|------|--------|------------|---------------|
| Degrees | ° | 1/360 of full rotation | Heading, bearing, attitude display |
| Degrees per second | °/s | Angular velocity | Gyroscope data, turn rate |
| Radians | rad | SI unit (2π = 360°) | Internal trigonometric calculations |

**Internal Storage:** Angles stored in radians for calculations, converted to degrees for display.

### 3.6 Time Units

| Unit | Symbol | Definition | Usage Context |
|------|--------|------------|---------------|
| Second | s | SI base unit | Timestamps, intervals |
| Millisecond | ms | 0.001 s | Sensor sampling, short intervals |
| Microsecond | µs | 0.000001 s | High-precision timing |

**Internal Storage:** Timestamps stored as Unix epoch (seconds since 01/01/1970 00:00:00 UTC) with millisecond precision where required.

### 3.7 Date and Time Format

**Standard Format:** UK format (dd/mm/yyyy or dd/mm/yy)

| Format | Example | Usage |
|--------|---------|-------|
| Date (full) | 02/04/2026 | Display, file metadata |
| Date (short) | 02/04/26 | Compact display |
| Time (24-hour) | 14:35:22 | Default display |
| Time (12-hour) | 2:35:22 PM | Optional display |
| DateTime | 02/04/2026 14:35:22 | Log entries, timestamps |
| Filename | 20260402_143522 | Log file naming (ISO 8601 sortable) |

**RTC Configuration:**
- Store time in UTC internally
- Apply timezone offset for display (+00:00 for GMT, +01:00 for BST)
- User-configurable timezone setting

### 3.8 Coordinate Units

| Unit | Symbol | Precision | Usage Context |
|------|--------|-----------|---------------|
| Decimal Degrees | DD | 6 decimal places (~0.1m) | Internal storage, GPS input |
| Degrees Minutes Seconds | DMS | 0.1" (~3m) | Optional display format |
| Degrees Decimal Minutes | DDM | 0.001' (~2m) | Aviation display format |

**Internal Storage:** Coordinates stored as decimal degrees (DD) with double precision.

**Example Formats:**
- DD: 51.507400, -0.127800
- DMS: 51°30'26.6"N, 0°07'40.1"W
- DDM: 51°30.444'N, 0°07.668'W

### 3.9 Temperature Units

| Unit | Symbol | Conversion | Usage |
|------|--------|------------|-------|
| Celsius | °C | SI derived | Default display, internal |
| Fahrenheit | °F | (°C × 9/5) + 32 | Optional display |

**Internal Storage:** Temperature stored in Celsius.

### 3.10 Unit Conversion Reference
```c
// Distance conversions
#define METRES_TO_KM        0.001
#define METRES_TO_NM        0.000539957
#define METRES_TO_FEET      3.28084
#define METRES_TO_MILES     0.000621371

// Speed conversions (from m/s)
#define MPS_TO_KMH          3.6
#define MPS_TO_MPH          2.23694
#define MPS_TO_KNOTS        1.94384
#define MPS_TO_FPM          196.850

// Angle conversions
#define DEG_TO_RAD          0.0174533
#define RAD_TO_DEG          57.2958

// Acceleration
#define G_TO_MPS2           9.80665
#define MPS2_TO_G           0.101972
```

---

## 4. Operating Modes

The system shall support three distinct operating modes with different algorithms, UI layouts, and default unit displays.

### 4.1 Pedestrian Mode
- Primary method: Step detection and counting
- Distance: Step count × configurable stride length
- Speed: Distance / elapsed time (averaged)
- Best accuracy, lowest drift
- **Default units:** km/h, km, metres
- **Assumed max acceleration:** ≤2g (per Section 2.1)

### 4.2 Vehicle Mode
- Primary method: Acceleration integration with turn detection
- Requires aggressive vibration filtering
- Benefits significantly from future GPS or OBD-II input
- 2D motion assumption (ground plane)
- **Default units:** km/h (or mph), km (or miles), metres
- **Assumed max acceleration:** ≤4g (per Section 2.1)

### 4.3 Aircraft Mode
- Primary method: Full 3D acceleration integration
- Tracks pitch, roll, and yaw attitudes
- Vertical speed estimation from Z-axis integration
- Highest sensor ranges enabled
- 3D position tracking
- **Default units:** knots, NM, feet (altitude), ft/min (vertical speed)
- **Assumed max acceleration:** ≤6g (per Section 2.1, turbulence/emergency only)

---

## 5. System Architecture

### 5.1 Software Modules
```
┌─────────────────────────────────────────────────────────┐
│                    Application Layer                     │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────────┐   │
│  │   Map UI    │ │ Data Display│ │ Settings/Config │   │
│  └─────────────┘ └─────────────┘ └─────────────────┘   │
├─────────────────────────────────────────────────────────┤
│                    Processing Layer                      │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────────┐   │
│  │  Position   │ │   Motion    │ │    Sensor       │   │
│  │  Estimator  │ │  Classifier │ │    Fusion       │   │
│  └─────────────┘ └─────────────┘ └─────────────────┘   │
├─────────────────────────────────────────────────────────┤
│                     Driver Layer                         │
│  ┌───────┐ ┌───────┐ ┌───────┐ ┌───────┐ ┌─────────┐  │
│  │ QMI   │ │ PCF   │ │ LCD   │ │ Touch │ │ SD Card │  │
│  │ 8658  │ │ 85063 │ │ST7789 │ │CST328 │ │  FAT    │  │
│  └───────┘ └───────┘ └───────┘ └───────┘ └─────────┘  │
├─────────────────────────────────────────────────────────┤
│                    Hardware Layer                        │
│         I2C Bus          │    SPI Bus    │    GPIO      │
└─────────────────────────────────────────────────────────┘
```

### 5.2 Data Flow
```
IMU (100Hz) → Filter → Transform → Integrate → Position
                ↓
RTC → Timestamp → Elapsed Time → Speed Calculation
                ↓
SD Card ← Data Logger ← Motion State
                ↓
Map Tiles → Renderer → LCD Display
```

---

## 6. Implementation Phases

### Phase 1: Foundation (Weeks 1-2)

**Target Completion:** 16/04/2026

**Objectives:**
- Establish development environment
- Implement sensor drivers
- Create basic display framework

**Tasks:**

1.1 **Development Setup**
- Install Pico VSCode extension or configure Thonny for MicroPython
- Clone Waveshare demo repository
- Verify all sensors respond correctly

1.2 **IMU Driver**
- Initialise QMI8658 over I2C
- Configure sample rate (100Hz recommended)
- Configure accelerometer range based on mode (±8g default, adequate for assumed ≤6g max)
- Configure gyroscope range based on mode
- Implement interrupt-driven or polled data acquisition
- Apply factory calibration values
- Store raw data in SI units (m/s², rad/s)

1.3 **RTC Driver**
- Initialise PCF85063
- Implement time get/set functions (UTC storage)
- Create elapsed time calculation utilities
- Handle time rollover correctly
- Implement timezone offset for display
- Format output in UK date format (dd/mm/yyyy)

1.4 **Display Framework**
- Initialise ST7789T3 over SPI
- Integrate LVGL library
- Create basic screen layouts for each mode
- Implement touch input handling

### Phase 2: Sensor Processing (Weeks 3-4)

**Target Completion:** 30/04/2026

**Objectives:**
- Implement signal conditioning
- Create sensor fusion algorithm
- Develop calibration routines

**Tasks:**

2.1 **Calibration System**
- Implement static calibration routine (device at rest, per assumption 2.4)
- Measure and store accelerometer bias offsets (m/s²)
- Measure and store gyroscope bias offsets (rad/s)
- Store calibration data to SD card with timestamp (dd/mm/yyyy HH:MM:SS)
- Load calibration on startup

2.2 **Signal Filtering**

*Low-Pass Filter (all modes):*
```c
// Simple IIR low-pass filter
filtered = alpha * new_sample + (1 - alpha) * filtered;
// alpha = dt / (RC + dt), where RC defines cutoff frequency
```

*Vibration Rejection (vehicle/aircraft):*
- Implement moving average filter (window: 10-20 samples)
- Add high-pass filter to remove DC bias drift
- Consider Kalman filter for optimal noise rejection
- Filter design assumes moderate vibration per Section 2.2

2.3 **Coordinate Frame Transformation**
- Transform body-frame accelerations to world frame
- Use rotation matrix derived from orientation estimate
- Critical for vehicles on hills and aircraft in any attitude

2.4 **Orientation Estimation**
- Implement complementary filter (simple, effective):
```c
// Complementary filter for pitch/roll
// angle in radians internally, display in degrees
angle = 0.98 * (angle + gyro * dt) + 0.02 * accel_angle;
```
- Or implement Madgwick/Mahony filter for better performance
- Note: Without magnetometer, yaw will drift over time

### Phase 3: Motion Algorithms (Weeks 5-6)

**Target Completion:** 14/05/2026

**Objectives:**
- Implement mode-specific motion detection
- Calculate speed and distance
- Handle error accumulation

**Tasks:**

3.1 **Pedestrian Mode: Step Detection**
```
Algorithm:
1. Apply low-pass filter to vertical acceleration
2. Detect peaks above dynamic threshold
3. Validate step timing (0.3s - 1.5s between steps)
4. Increment step counter
5. Distance = steps × stride_length (metres, per assumption 2.3)
6. Speed = distance / elapsed_time (m/s, display as km/h)
```

3.2 **Vehicle Mode: Integration with Turn Detection**
```
Algorithm:
1. Apply vibration filter to acceleration data
2. Transform to world frame using orientation
3. Remove gravity component (9.80665 m/s²)
4. Detect stationary periods (zero-velocity updates)
5. Integrate acceleration to velocity (m/s)
6. Integrate velocity to position (metres)
7. Detect turns via gyroscope Z-axis (rad/s)
8. Apply ZUPT corrections when stationary detected
```

*Zero-Velocity Update (ZUPT):*
- Detect when acceleration magnitude ≈ 1g (9.81 m/s²) and gyro ≈ 0
- Reset velocity to zero to prevent drift
- Critical for vehicle applications at stops

3.3 **Aircraft Mode: 3D Integration**
```
Algorithm:
1. Use full 3D orientation from sensor fusion
2. Transform acceleration to NED (North-East-Down) frame
3. Remove gravity (accounting for attitude)
4. Integrate in 3D (all values in metres)
5. Track altitude changes separately (within assumed range per 2.2)
6. Calculate vertical speed from Z integration (m/s, display as ft/min)
```

3.4 **Error Estimation**
- Track time since last correction (GPS fix or ZUPT)
- Estimate position uncertainty radius (metres)
- Display uncertainty on map as growing circle
- Use drift rates from Section 2.5 for estimation

### Phase 4: Map System (Weeks 7-8)

**Target Completion:** 28/05/2026

**Objectives:**
- Implement SD card map storage
- Create map rendering system
- Enable position visualisation

**Tasks:**

4.1 **Map Data Format**
```
Directory structure on SD card:
/maps/
  /region_name/
    metadata.json    // Bounds, scale, tile size
    /tiles/
      tile_0_0.bmp   // Row 0, Column 0
      tile_0_1.bmp
      ...
```

*metadata.json format:*
```json
{
  "name": "Local Area",
  "created": "02/04/2026",
  "origin_lat": 51.5074,
  "origin_lon": -0.1278,
  "scale": 1000,
  "tile_size_px": 240,
  "tile_size_m": 500,
  "rows": 10,
  "cols": 10,
  "units": {
    "distance": "metres",
    "coordinates": "decimal_degrees"
  }
}
```

4.2 **Tile Management**
- Load tiles on demand based on current position
- Cache adjacent tiles for smooth scrolling
- Handle tile boundaries gracefully
- Memory management: Keep max 4 tiles loaded

4.3 **Position Rendering**
- Draw current position marker on map
- Show heading arrow (if orientation available)
- Draw breadcrumb trail (last N positions)
- Show uncertainty circle based on error estimate (radius in metres)

4.4 **User Interaction**
- Touch to set starting position on map (per assumption 2.4)
- Pinch to zoom (or button-based zoom)
- Pan by dragging
- Long-press for context menu (set origin, clear trail)

### Phase 5: Data Logging (Week 9)

**Target Completion:** 04/06/2026

**Objectives:**
- Log motion data for analysis
- Enable track export
- Support flight/trip review

**Tasks:**

5.1 **Log Format**
```csv
# Motion Tracker Log
# Created: 02/04/2026 14:35:22 UTC
# Mode: Vehicle
# Units: timestamp(UTC), ax(m/s²), ay(m/s²), az(m/s²), gx(rad/s), gy(rad/s), gz(rad/s), lat(deg), lon(deg), alt(m), speed(m/s), heading(deg)
timestamp,ax,ay,az,gx,gy,gz,lat_est,lon_est,alt_est,speed,heading
1712068522.123,0.15,-0.02,9.78,0.001,-0.002,0.015,51.507400,-0.127800,45.2,12.5,45.0
```

5.2 **Logging Control**
- Start/stop via UI button
- Auto-start option based on motion detection
- Configurable sample rate (1Hz - 100Hz)
- File naming: YYYYMMDD_HHMMSS_mode.csv (ISO 8601 for sortability)
- Display filename with UK date format in UI
- Assumes SD card present per Section 2.4

5.3 **Track Playback**
- Load previous track from SD
- Replay on map with time slider
- Show statistics (max speed, distance, duration)
- Display dates in UK format (dd/mm/yyyy)

### Phase 6: GPS Integration Preparation (Week 10)

**Target Completion:** 11/06/2026

**Objectives:**
- Create abstraction layer for position sources
- Reserve hardware interfaces
- Design fusion algorithm

**Tasks:**

6.1 **Position Source Interface**
```c
typedef struct {
    double latitude;      // Decimal degrees
    double longitude;     // Decimal degrees
    double altitude;      // Metres above MSL
    float speed;          // m/s
    float heading;        // Degrees (0-360)
    float accuracy;       // Metres (CEP)
    uint32_t timestamp;   // Unix epoch (UTC)
    bool valid;
} position_t;

typedef struct {
    bool (*init)(void);
    bool (*get_position)(position_t *pos);
    void (*deinit)(void);
} position_source_t;
```

6.2 **GPS Module Planning**
- Reserve UART pins (TX/RX) for GPS module
- Recommended modules: u-blox NEO-6M, NEO-M8N, or BN-220
- Plan for NMEA parsing (GGA, RMC sentences)
- Consider GPS antenna placement requirements

6.3 **Sensor Fusion Strategy**
- GPS provides absolute position at 1-10Hz
- IMU provides relative updates at 100Hz
- When GPS available: Reset position, correct drift
- When GPS unavailable: Continue with IMU dead reckoning
- Implement Extended Kalman Filter for optimal fusion
- Review assumptions per Section 2.7 when GPS integrated

---

## 7. User Interface Design

### 7.1 Main Screen Layout
```
┌────────────────────────────────────────┐
│ Mode: Vehicle  02/04/26 14:35  78%    │  <- Status bar (UK date)
├────────────────────────────────────────┤
│                                        │
│          MAP DISPLAY AREA              │
│             240 x 200                  │
│               ┼ ←Position marker       │
│                                        │
├────────────────────────────────────────┤
│ Speed: 45.2 km/h   Dist: 12.4 km      │  <- Data bar
│ Time: 00:15:32     Heading: 045°      │
├────────────────────────────────────────┤
│ [Start] [Mode] [Settings] [Log]       │  <- Control buttons
└────────────────────────────────────────┘
```

### 7.2 Settings Screen

**General Settings:**
- Operating mode selection (Pedestrian/Vehicle/Aircraft)
- Display brightness

**Units Settings:**
- Distance units: Metric (km/m) / Imperial (miles/ft) / Nautical (NM/ft)
- Speed units: km/h / mph / knots
- Altitude units: Metres / Feet
- Vertical speed: m/s / ft/min
- Temperature: Celsius / Fahrenheit
- Coordinate format: DD / DMS / DDM

**Date/Time Settings:**
- Date format: UK (dd/mm/yyyy) / US (mm/dd/yyyy) / ISO (yyyy-mm-dd)
- Time format: 24-hour / 12-hour
- Timezone: UTC offset (-12 to +14)
- Auto-DST: On/Off (for British Summer Time)

**Mode-Specific Settings:**
- Stride length (pedestrian mode): 0.50m - 1.20m
- Sensor ranges: Auto / Manual selection

**System Settings:**
- Calibration trigger
- GPS enable (when hardware present)
- Map selection
- Factory reset

### 7.3 Audio Feedback

- Beep on mode change
- Speed alerts (configurable threshold)
- Low battery warning
- GPS fix acquired/lost (future)

---

## 8. Configuration Parameters

### 8.1 Sensor Configuration

| Parameter | Pedestrian | Vehicle | Aircraft | Units |
|-----------|------------|---------|----------|-------|
| Accel Range | ±4g | ±8g | ±8g* | g (×9.81 m/s²) |
| Gyro Range | ±256°/s | ±256°/s | ±512°/s | °/s |
| Sample Rate | 50Hz | 100Hz | 100Hz | Hz |
| Filter Cutoff | 5Hz | 2Hz | 10Hz | Hz |

*Note: ±8g sufficient for human-occupied aircraft per assumption 2.1. ±16g available if needed.

### 8.2 Algorithm Parameters

| Parameter | Default | Range | Units | Description |
|-----------|---------|-------|-------|-------------|
| stride_length | 0.75 | 0.5-1.2 | m | Step length for pedestrian mode |
| zupt_threshold | 0.05 | 0.01-0.1 | g | Stationary detection sensitivity |
| filter_alpha | 0.1 | 0.01-0.5 | - | Low-pass filter coefficient |
| complementary_alpha | 0.98 | 0.9-0.99 | - | Orientation filter coefficient |
| uncertainty_growth | 0.1 | 0.01-1.0 | m/s | Position uncertainty accumulation rate |

### 8.3 Display Unit Defaults by Mode

| Parameter | Pedestrian | Vehicle | Aircraft |
|-----------|------------|---------|----------|
| Speed | km/h | km/h | knots |
| Distance | km | km | NM |
| Altitude | m | m | ft |
| Vertical Speed | - | - | ft/min |

---

## 9. Future Enhancements

### 9.1 Hardware Additions

| Component | Purpose | Interface | Priority |
|-----------|---------|-----------|----------|
| GPS Module | Absolute position | UART | High |
| Magnetometer | Heading reference | I2C | Medium |
| Barometer | Altitude accuracy | I2C | Medium (aircraft) |
| OBD-II Adapter | Vehicle speed reference | UART/CAN | Medium (vehicle) |
| External Antenna | GPS reception | - | Low |

### 9.2 Software Features

- Route planning and navigation
- Geofencing alerts
- Wireless data export (if WiFi/BLE module added)
- Multi-track comparison
- Weather overlay on maps
- Integration with flight logging standards (IGC format for gliders)

### 9.3 Accuracy Improvements

- Machine learning for motion classification
- Adaptive filter tuning based on detected motion type
- Map matching (snap position to known roads)
- Collaborative positioning (multiple units)

---

## 10. Known Limitations and Mitigations

| Limitation | Impact | Mitigation |
|------------|--------|------------|
| No magnetometer | Heading drifts over time | Use GPS heading when available; prompt user for heading reset |
| IMU drift | Position error grows ~1-10m per minute | Implement ZUPT; add GPS; show uncertainty to user |
| No barometer | Altitude accuracy poor | Rely on GPS altitude; or add external barometer |
| ±8g accel limit | May clip in extreme turbulence | Adequate for human-occupied per assumptions; ±16g available |
| Small display | Limited map visibility | Implement zoom; allow external display (future) |
| Battery life | Limited operation time | Implement sleep modes; reduce sample rate when stationary |

---

## 11. Development Resources

### 11.1 Documentation

- [Waveshare Wiki - RP2350-Touch-LCD-2.8](https://www.waveshare.com/wiki/RP2350-Touch-LCD-2.8)
- [QMI8658 Datasheet](https://files.waveshare.com/wiki/common/QMI8658A_Datasheet_Rev_A.pdf)
- [PCF85063 Datasheet](https://files.waveshare.com/wiki/common/PCF85063A.pdf)
- [ST7789T3 Datasheet](https://files.waveshare.com/wiki/common/ST7789T3_SPEC_V1.0.pdf)
- [RP2350 Datasheet](https://files.waveshare.com/wiki/common/Rp2350-datasheet.pdf)

### 11.2 Demo Code

- Waveshare Demo Package (includes IMU, RTC, LVGL examples)
- Raspberry Pi Pico SDK examples
- LVGL documentation and examples

### 11.3 Libraries

- LVGL (graphics)
- FatFS (SD card filesystem)
- Pico SDK (hardware abstraction)

---

## 12. Testing Plan

### 12.1 Unit Tests

- Sensor read/write verification
- Filter coefficient validation
- Coordinate transformation accuracy
- Time calculation correctness
- Unit conversion accuracy (all conversions in Section 3)

### 12.2 Integration Tests

- Sensor fusion stability over time
- Map tile loading and rendering
- Touch response accuracy
- Data logging integrity
- Date/time format correctness (UK format throughout)

### 12.3 Field Tests

| Test | Mode | Duration | Success Criteria | Target Date |
|------|------|----------|------------------|-------------|
| Walking circuit | Pedestrian | 1 hour | <5% distance error | 20/05/2026 |
| City driving | Vehicle | 30 min | <20% distance error without GPS | 27/05/2026 |
| Highway driving | Vehicle | 1 hour | Position on correct road | 03/06/2026 |
| Light aircraft circuit | Aircraft | 20 min | Return to start within 500m | 10/06/2026 |

Note: All field tests assume normal human-occupied operation per Section 2 assumptions.

---

## Appendix A: Pin Assignments

Refer to Waveshare documentation for complete pinout. Key interfaces:

- **I2C (Sensors):** SDA, SCL for QMI8658, PCF85063, CST328
- **SPI (Display):** MOSI, SCK, CS, DC, RST for ST7789T3
- **SPI (SD Card):** Shared or dedicated SPI bus
- **UART (Future GPS):** TX, RX reserved
- **PWM (Speaker):** Audio output pin

---

## Appendix B: Glossary

| Term | Definition |
|------|------------|
| AHRS | Attitude and Heading Reference System |
| BST | British Summer Time (UTC+1) |
| CEP | Circular Error Probable (GPS accuracy measure) |
| Dead Reckoning | Position estimation from motion sensors |
| DD | Decimal Degrees (coordinate format) |
| DDM | Degrees Decimal Minutes (coordinate format) |
| DMS | Degrees Minutes Seconds (coordinate format) |
| g | Standard gravity (9.80665 m/s²) |
| GMT | Greenwich Mean Time (UTC+0) |
| IMU | Inertial Measurement Unit |
| MSL | Mean Sea Level (altitude reference) |
| NED | North-East-Down coordinate frame |
| NM | Nautical Mile (1,852 metres) |
| NMEA | National Marine Electronics Association (GPS protocol) |
| SI | International System of Units |
| UTC | Coordinated Universal Time |
| ZUPT | Zero Velocity Update (drift correction technique) |

---

## Appendix C: Unit Conversion Quick Reference

### Distance
| From | To km | To miles | To NM | To feet |
|------|-------|----------|-------|---------|
| 1 metre | 0.001 | 0.000621 | 0.00054 | 3.281 |
| 1 km | 1 | 0.621 | 0.540 | 3,281 |
| 1 mile | 1.609 | 1 | 0.869 | 5,280 |
| 1 NM | 1.852 | 1.151 | 1 | 6,076 |

### Speed
| From | To km/h | To mph | To knots | To m/s |
|------|---------|--------|----------|--------|
| 1 m/s | 3.6 | 2.237 | 1.944 | 1 |
| 1 km/h | 1 | 0.621 | 0.540 | 0.278 |
| 1 mph | 1.609 | 1 | 0.869 | 0.447 |
| 1 knot | 1.852 | 1.151 | 1 | 0.514 |

### Vertical Speed
| From | To ft/min | To m/s |
|------|-----------|--------|
| 1 m/s | 196.85 | 1 |
| 100 ft/min | 100 | 0.508 |

### Acceleration
| From | To m/s² | To g |
|------|---------|------|
| 1 g | 9.80665 | 1 |
| 1 m/s² | 1 | 0.102 |

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 02/04/2026 | - | Initial draft |
| 1.1 | 02/04/2026 | - | Added Units and Measurements section; UK date format throughout |
| 1.2 | 02/04/2026 | - | Added Assumptions section (Section 2); revised sensor ranges based on human occupancy assumption |

---

*Document prepared for RP2350-Touch-LCD-2.8 motion tracking project development.*
*All dates in UK format (dd/mm/yyyy). All internal calculations use SI units.*
*Design assumes human-occupied scenarios with acceleration ≤6g maximum.*
