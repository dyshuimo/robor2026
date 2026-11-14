# PS4 field-centric mini swerve

Simple Java `TimedRobot` code for four swerve modules, each with a NEO Vortex
drive motor and a NEO (REV-21-1650) steering motor, both on SPARK MAX controllers.
This project uses **WPILib 2025.3.2 and REVLib 2025.0.3** despite the folder name.
Use matching 2025 tools and controller firmware for this project.

The PS4 controller connects to the **Driver Station computer**, which sends
its inputs to the roboRIO. Connect it by USB and place it in joystick slot **0**
in the Driver Station's USB tab. Verify the axes/buttons there before enabling.

## CAN IDs

| Module | Drive | Steering |
| --- | --- | --- |
| Front left | 1 | 3 |
| Front right | 4 | 6 |
| Back left | 7 | 9 |
| Back right | 10 | 12 |

The rear IDs are inferred from the pattern; verify them in REV Hardware Client.
No device is assigned to IDs 2, 5, 8, or 11 by this code.

## Required setup

Edit `src/main/java/frc/robot/Constants.java`:

1. Set `WHEELBASE_METERS`: front-to-back distance between steering pivot centers.
2. Set `TRACKWIDTH_METERS`: left-to-right distance between steering pivot centers.
3. Set `STEER_GEAR_RATIO`: steering **motor rotations per full module rotation**.
   For example, a 12.8:1 steering gearbox would use `12.8`, but use your module's
   actual ratio. The NEO's part number 1650 does not give this ratio.
4. Set `WHEEL_DIAMETER_METERS` and `DRIVE_GEAR_RATIO` (drive motor rotations per
   wheel rotation). These convert the drive encoders' rotations into meters.
5. Select the **actual installed gyro** using `GYRO_TYPE` and `GYRO_SPI_PORT`.
   `RobotGyro.java` includes WPILib drivers for ADIS16470 (Z axis vertical) and
   ADXRS450 over SPI. The default is `NOT_CONFIGURED` because the gyro model has
   not been provided. Pigeon or navX requires its vendor dependency and the
   matching implementation in `RobotGyro.java`; selecting ADIS will not read one.
6. Verify the CAN IDs, motor inversion flags, and encoder signs. Arrays are ordered
   front left, front right, back left, back right.

The five unknown measurements default to zero. The code keeps all motor outputs
off until they are positive, a gyro is configured and connected, and the steering
references are zeroed. Field-centric control cannot work with only the motor
encoders; it needs a measured robot heading.

Each NEO's sensor cable must connect to its SPARK MAX encoder port. A Vortex needs
the [NEO Vortex Solo Adapter](https://docs.revrobotics.com/brushless/neo/vortex/solo-adapter)
to connect to a SPARK MAX. This code uses the NEOs' built-in **relative** encoders
for drive distance and steering angle, together with a gyro for heading. It does
not use separate absolute steering encoders.

## Align and drive

1. Support the chassis securely with all wheels off the floor for initial tests.
2. Deploy the code, connect Driver Station, and keep the robot **disabled**.
   Keep the chassis still during the gyro's startup calibration.
3. Manually align all four modules to the same forward reference. Mark that
   orientation so it is repeatable; a module turned 180 degrees is not the same
   zero, even though its wheel looks parallel. Steering uses coast mode to allow
   alignment while disabled.
4. Press and release **Options** while disabled. Wait for each module's
   `Swerve/<module>/Zeroed` to become true. This records the steering positions as zero;
   pressing the button does not move the modules into alignment for you.
5. Before enabling, turn each module slightly counterclockwise as viewed from
   above. Its dashboard angle must increase. If it decreases, change that module's
   `STEER_ENCODER_SIGN` to `-1.0`, redeploy, and align/zero again. A full module turn
   should change the angle by 360 degrees; otherwise check the steering ratio.
6. Check the drive distance signs by rolling each wheel in its marked forward
   direction while disabled. Its `Drive meters` value must increase. Adjust
   `DRIVE_ENCODER_SIGN` if needed. A full wheel rotation should measure one wheel
   circumference; check the drive gear ratio and wheel diameter if it does not.
7. Return the modules to the marked forward positions and press Options again.
   Point the chassis in the direction you want to call field-forward, center both
   sticks, and press **Triangle**. Wait for `Swerve/Ready` to become true.
8. Verify that turning the chassis counterclockwise increases `Swerve/Heading degrees`.
   Set `GYRO_REVERSED` if the heading has the opposite sign, then redeploy and repeat
   calibration. Enable **Teleop** and gently test the sticks. No L1 button is needed.

| Control | Action |
| --- | --- |
| Left stick, any direction | Translate in that direction relative to the field |
| Right stick left/right | Rotate counterclockwise/clockwise |
| Center both sticks | Stop drive and steering |
| Triangle, disabled or Teleop | Reset odometry to (0, 0) and current heading to 0 degrees |
| Options while disabled | Record all aligned modules as zero |

Controls are **field-centric**: pushing the left stick up keeps moving in the same
field direction even while the right stick spins the chassis. Triangle defines
the current facing as the new field-forward direction and clears the measured X/Y
position. It does not physically turn the robot, re-zero the steering modules, or
reset hardware encoders. Center the sticks before resetting the reference.

The starting drive and steering limits are both 20% output. This is not a fixed
speed in meters per second. On blocks, verify forward drive direction for every
module. Change its `DRIVE_INVERTED` flag if needed. With encoder signs verified,
if steering moves away from its requested angle, stop and change that module's
`STEER_INVERTED` flag. Increase `STEER_KP` gradually if steering is too weak, or
reduce it if it oscillates. Align/zero after every redeploy or robot/controller
power cycle. A detected drive or steering controller reset invalidates calibration.
After a lost sensor reference, restore the sensor, align/zero if a motor controller
restarted, and press Triangle to establish a new pose. Driving remains off until
the reference is valid.

## How the code works

- `Robot.java` reads the sticks, applies a deadband, and calls `swerve.drive()`.
- `SwerveDrive.java` rotates field joystick inputs into the robot frame using the
  measured heading, adds wheel translation and rotation vectors, and scales all
  four speeds together when necessary.
- `SwerveModule.java` compares the requested angle to the measured steering angle.
  A simple proportional controller turns the wheel using the shortest path.
  It reverses the drive motor when that avoids steering more than 90 degrees,
  and reduces drive output while the wheel is turning into position.
- `SwervePoseTracker.java` uses WPILib `SwerveDriveOdometry` with actual wheel travel
  and gyro readings. Triangle resets it with the current sensor baselines so the
  next update does not add old travel back into the pose. The same heading
  reference is used for driving and the dashboard.
- `RobotGyro.java` reads the selected heading sensor, reporting counterclockwise
  turns as positive.
- `Constants.java` contains the hardware measurements and tuning settings.

The motors stop when the sticks are centered; this version does not hold a steering
angle while stopped. Open-loop drive is suitable for an initial driving test.
Odometry updates in every robot loop, including disabled mode, and is displayed
as `Swerve/X meters`, `Swerve/Y meters`, `Swerve/Heading degrees`, and `Swerve/Field`.
It can drift with wheel slip. Autonomous path following and closed-loop drive
velocity control are not included.

## Build and deploy

Use the WPILib 2025 VS Code commands **Build Robot Code** and **Deploy Robot Code**,
or run with Java 17:

```sh
./gradlew build
./gradlew deploy
```

The configured team number is 9062; change `.wpilib/wpilib_preferences.json` if needed.
`vendordeps/REVLib.json` installs REVLib through Gradle. Tests run with
`./gradlew test` and check forward, reverse, strafe, rectangular-chassis rotation,
combined motion normalization, and zero input. Hardware direction and tuning
still need the on-block checks above. Additional tests verify field directions at
different headings, odometry in meters, Triangle resets, and sensor-reference loss.

## Editor diagnostics

The project compiles with the WPILib 2025 Java 17 JDK. The saved VS Code diagnostics
inspected during this change contained a missing-explicit-encoding warning and
hardware `TODO` tasks, rather than Java compile errors. The Gradle compiler and
VS Code files now explicitly use UTF-8. The `TODO`s identify measurements you
still need to supply; they do not prevent compilation.

If imports are red after adding a vendor dependency, run **Java: Reload Projects**
from the command palette. If necessary, run **Java: Clean Java Language Server
Workspace** and let the project re-import. Open the whole project folder, and use
WPILib's Java 17 for Gradle; the system Java on this computer is Java 26.

On this Mac, the verified build command is:

```sh
JAVA_HOME=/Users/kimi/wpilib/2025/jdk ./gradlew build
```

References: [WPILib PS4Controller](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/PS4Controller.html),
[REV SPARK configuration](https://docs.revrobotics.com/revlib/spark/configuring-a-spark),
[WPILib swerve odometry and pose reset](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html),
[WPILib field-relative coordinates](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html).
