# 2026 Mini Swerve Project
### Created by Frank (***dyshuimo*** / ***import-frankz***)
**FRC 9062 Critical Circuits**

Robotics project built with **REV SPARK MAX**, **CTRE Absolute CANcoder**, with addtional electronics. Programed to be only a drive base, and is capable with creating and operating **Autonomous Commands**.

## Project Overview 📄

This project is based on a drive base. The code enables testings, with simulation of build of robot functions. The swerve drives are designed to be small, with the electronics within the robot using up most of the space. Functionally, this project is a practice of building robot, and understading the basis of robot development. Which in the future, updates and more advanced functions could be added or tested on this robot.

---
### Code Design:

```
                        ┌─────────────────────────────────────────────────────────────┐
                        │                            Main                             │
                        │              RobotBase.startRobot(Robot::new)               │
                        └──────────────────────────────┬──────────────────────────────┘
                                                       ▼
                        ┌─────────────────────────────────────────────────────────────┐
                        │                     Robot (TimedRobot)                      │
                        │   PS4 input, startup/mode lifecycle, Triangle pose reset    │
                        └──────────────────────────────┬──────────────────────────────┘
                                                       ▼
                        ┌─────────────────────────────────────────────────────────────┐
                        │                         SwerveDrive                         │
                        │    Field-relative drive, teleop alignment, sensor checks    │
                        │             SmartDashboard + Field2d telemetry              │
                        └──────────────────────────────┬──────────────────────────────┘
                                                       │
                 ┌─────────────────────────────────────┼─────────────────────────────────────┐
                 ▼                                     ▼                                     ▼
┌─────────────────────────────────┐   ┌─────────────────────────────────┐   ┌─────────────────────────────────┐
│ SwerveModule x4 (FL/FR/BL/BR)   │   │ RobotGyro                       │   │ SwervePoseTracker               │
│ • Open-loop drive, P steering   │   │ • Redux Boron CANandgyro        │   │ • Wheel positions + heading     │
│ • 2x SPARK MAX per module       │   │ • CAN bus (ID 0)                │   │ • SwerveDriveKinematics         │
│ • Relative drive encoder        │   │ • Connection/calibration checks │   │ • SwerveDriveOdometry           │
│ • AbsoluteSteeringEncoder       │   │ • CCW-positive heading          │   │ • Pose2d and origin reset       │
│   └─ CanCoderSteeringEncoder    │   │                                 │   │                                 │
│      (CTRE CANcoder)            │   │                                 │   │                                 │
└─────────────────────────────────┘   └─────────────────────────────────┘   └─────────────────────────────────┘
```
---
### Project File Structure:

```
src/
├── main/
│   ├── deploy/                             # Optional files deployed to the roboRIO
│   └── java/frc/robot/
│       ├── Constants.java                   # CAN IDs, dimensions, calibration, gains & limits
│       ├── Main.java                        # Standard WPILib robot entrypoint
│       ├── Robot.java                       # TimedRobot lifecycle, PS4 controls & pose reset
│       └── subsystems/
│           └── swerve/
│               ├── RobotGyro.java           # Redux Boron CANandgyro heading reads
│               ├── SwerveDrive.java         # Field-relative drive, alignment & dashboard telemetry
│               ├── SwerveModule.java        # SPARK MAX control & nested CANcoder encoder adapter
│               └── SwervePoseTracker.java   # Measured wheel/gyro odometry & origin reset
└── test/java/frc/robot/
    ├── RobotStartupTest.java                # Robot startup & missing-sensor interlocks in simulation
    └── subsystems/
        └── swerve/
            ├── SwerveDriveTest.java         # Wheel vectors, normalization & field-relative controls
            ├── SwerveModuleTest.java        # CANcoder validation, simulation & forward alignment
            ├── SwervePoseTrackerTest.java   # Odometry baselines, reset & sensor-reference loss
            ├── SwerveStartupTest.java       # Delayed sensors, teleop alignment & startup recovery
            └── TestSteeringEncoder.java     # Deterministic steering samples for module/drive tests
```

## Hardware Maps 🛞

### Devices on robot

- **Each** of all four **Drives** include the elements listed:
    - Drive Motor: *Neo Vortex Brushless Motor*
    - Steer Motor: *Neo V1.1 1650 Brushless Motor*
    - Encoder: *CTRE Absolute Magnetic CANcoder*

- **Power Distribution Module**: *CTRE Power Distribution Panel* **and** *REV Mini Power Module*
- **Robot Mainboard**: *NI roboRIO*
- **Radio**: *OpenMesh OM5P-AC* , **powered by** *REV Robotics Radio Power Module*
- **Gyro**: *Redux Robotics Boron CANandgyro*

The motors are connected to **Motor Controllers**, which all motors are connected through the *Power Distribution Module*, as well as the **roboRIO** and the *Mini Power Module*. All **CANcoders**, **Gyro**, and **Radio Modules** are powered through the *Mini Power Module*.

---

### CAN devices

| CAN ID | Device Type | Controller / Model | Subsystem / Function | Bus |
|---|---|---|---|---|
| **0** | Gyro | Redux Robotics Boron CANandgyro | Swerve / robot heading | RIO CAN |
| **1** | Motor controller | REV SPARK MAX | Swerve / front left drive | RIO CAN |
| **2** | Absolute encoder | CTRE CANcoder | Swerve / front left steering angle | CANivore / RIO CAN |
| **3** | Motor controller | REV SPARK MAX | Swerve / front left steering | RIO CAN |
| **4** | Motor controller | REV SPARK MAX | Swerve / front right drive | RIO CAN |
| **5** | Absolute encoder | CTRE CANcoder | Swerve / front right steering angle | CANivore / RIO CAN |
| **6** | Motor controller | REV SPARK MAX | Swerve / front right steering | RIO CAN |
| **7** | Motor controller | REV SPARK MAX | Swerve / back left drive | RIO CAN |
| **8** | Absolute encoder | CTRE CANcoder | Swerve / back right steering angle | CANivore / RIO CAN |
| **9** | Motor controller | REV SPARK MAX | Swerve / back left steering | RIO CAN |
| **10** | Motor controller | REV SPARK MAX | Swerve / back right drive | RIO CAN |
| **11** | Absolute encoder | CTRE CANcoder | Swerve / back left steering angle | CANivore / RIO CAN |
| **12** | Motor controller | REV SPARK MAX | Swerve / back right steering | RIO CAN |

---

### Controller Mappings

| Input | Action | Behavior |
|---|---|---|
| **Left Stick Y** | Translation Forward / Back | Teleop: field-relative forward/back movement; 8% deadband. |
| **Left Stick X** | Translation Left / Right | Teleop: field-relative left/right movement; 8% deadband. |
| **Right Stick X** | Chassis Rotation | Teleop: left rotates counterclockwise, right clockwise; 8% deadband. |
| **Triangle** | Reset Heading | Teleop/disabled: zeroes position and heading; waits for valid sensors. |
| **(Default / Idle)** | Align Wheels Forward | Teleop idle: drive stops; steering keeps wheels facing robot-forward. |


## Building, Testing & Deploying 🛠️

### Build Project
```bash
./gradlew build
```

### Run Unit Tests (23 tests)
```bash
./gradlew test
```

### Run WPILib Desktop Simulation
```bash
./gradlew simulateJava
```

### Deploy to Robot
```bash
./gradlew deploy
```
