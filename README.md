# MargieRecode - FRC Robot Project

An FRC (FIRST Robotics Competition) robot project using WPILib 2025, AdvantageKit for logging, and CTRE Phoenix 6 for motor control. The project follows a command-based architecture with IO abstraction layers for hardware components.

## Features

- **Swerve Drive** - Full swerve drive system with module control
- **AdvantageKit Logging** - Comprehensive logging and log replay support
- **IO Abstraction** - Hardware-independent subsystem design for easy testing and simulation
- **PathPlanner Integration** - Autonomous path following
- **LimeLight** - Camera and vision processing for target tracking
- **Elastic Dashboard** - Using Elastic dashboard for logging and competition purposes instead of Shuffleboard or Driverstation.

## Prerequisites

- WPILib 2025 installed (includes Java JDK 17)
- Visual Studio Code with WPILib extension (recommended)

## Build Commands

```bash
# Build the project
./gradlew build

# Run tests
./gradlew test

# Format code (Spotless)
./gradlew spotlessApply

# Check formatting
./gradlew spotlessCheck

# Deploy to RoboRIO
./gradlew deploy

# Run simulation
./gradlew simulateJava

# Log replay
./gradlew replayWatch
```

**Note:** On Windows, if JAVA_HOME is not configured, use:
```bash
JAVA_HOME="C:/Users/Public/wpilib/2025/jdk" ./gradlew build
```

## Project Structure

```
src/main/java/frc/robot/
├── Main.java           # Entry point
├── Robot.java          # LoggedRobot, initializes AdvantageKit
├── RobotContainer.java # Subsystems, commands, button bindings
├── Constants.java      # Global constants
├── subsystems/
│   ├── drive/          # Swerve drive system
│   ├── intake/         # Game piece intake
│   ├── shooter/        # Shooting mechanism
│   ├── superstructure/ # Coordinated subsystem control
│   └── vision/         # Camera and vision processing
└── commands/           # Robot commands
```

## Architecture

### IO Abstraction Pattern

Each subsystem follows an IO abstraction pattern for hardware independence:

- **Subsystem Class** - Core logic and command interface (e.g., `Drive.java`)
- **IO Interface** - Hardware abstraction layer (e.g., `DriveIO.java`)
- **IO Implementations**:
  - Real hardware (e.g., `DriveIOTalonFX.java`)
  - Simulation (e.g., `DriveIOSim.java`)

This pattern enables:
- Unit testing without hardware
- Full simulation support
- Easy hardware swaps
- AdvantageKit log replay

## Vendor Dependencies

- WPILib 2025.3.2
- CTRE Phoenix 6
- AdvantageKit
- PathPlanner
- PhotonLib
- Maple-sim

## CTRE Phoenix 6 Notes

Phoenix 6 uses typed units, not raw doubles:

```java
// Correct usage
StatusSignal<Angle> positionSignal = motor.getPosition();
double rotations = positionSignal.getValue().in(Units.Rotations);

// Incorrect - won't compile
StatusSignal<Double> positionSignal = motor.getPosition();
```

Current limits use `SupplyCurrentLimit` and `StatorCurrentLimit` (not threshold-based limits).
