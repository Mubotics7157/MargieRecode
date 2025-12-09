# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FRC (FIRST Robotics Competition) robot project using WPILib 2025, AdvantageKit for logging, and CTRE Phoenix 6 for motor control. The project follows a command-based robot architecture with IO abstraction layers for hardware components.

## Build and Development Commands

### Building the Project
```bash
# Windows with WPILib's JDK
JAVA_HOME="C:/Users/Public/wpilib/2025/jdk" ./gradlew build

# Or if JAVA_HOME is already configured
./gradlew build
```

### Running Tests
```bash
./gradlew test
```

### Code Formatting (Spotless)
```bash
./gradlew spotlessApply  # Auto-format code
./gradlew spotlessCheck  # Check formatting
```

### Deploy to RoboRIO
```bash
./gradlew deploy
```

### Simulation
```bash
./gradlew simulateJava  # Run robot simulation
```

### AdvantageKit Log Replay
```bash
./gradlew replayWatch  # Watch log replay
```

## Architecture Overview

### Core Structure
- **Main Entry**: `frc.robot.Main` - Entry point, launches Robot class
- **Robot Class**: `frc.robot.Robot` - Extends LoggedRobot, initializes AdvantageKit logging
- **RobotContainer**: `frc.robot.RobotContainer` - Central configuration for subsystems, commands, and button bindings
- **Constants**: `frc.robot.Constants` - Global robot constants and configuration

### Subsystem Architecture
Each subsystem follows an IO abstraction pattern for hardware independence:
- **Subsystem Class**: Core logic and command interface (e.g., `Drive.java`)
- **IO Interface**: Hardware abstraction layer (e.g., `DriveIO.java`)
- **IO Implementations**:
  - Real hardware (e.g., `DriveIOTalonFX.java`)
  - Simulation (e.g., `DriveIOSim.java`)

Current subsystems:
- `drive/` - Swerve drive system with module control
- `intake/` - Game piece intake mechanism
- `shooter/` - Shooting mechanism
- `superstructure/` - Coordinated subsystem control
- `vision/` - Camera and vision processing

### Command Structure
- Commands in `frc.robot.commands/`
- Uses WPILib's command-based framework
- Factory methods in `DriveCommands.java` and `SuperstructureCommands.java`

## Critical Implementation Details

### CTRE Phoenix 6 API Usage
Phoenix 6 uses typed units, not raw doubles:
```java
// CORRECT
StatusSignal<Angle> positionSignal = motor.getPosition();
double rotations = positionSignal.getValue().in(Units.Rotations);

// INCORRECT - Won't compile
StatusSignal<Double> positionSignal = motor.getPosition();
```

### Current Limits with Phoenix 6
Use `SupplyCurrentLimit` and `StatorCurrentLimit`, not threshold-based limits.

### AdvantageKit Logging
- All IO operations must log inputs/outputs through AutoLog
- Use `@AutoLog` annotation on IO classes
- Logger is initialized in Robot constructor

### Gradle Configuration
- Java 17 target
- Spotless auto-formatting runs before compilation
- Event branches auto-commit on deploy
- JVM heap limited to 100MB for RoboRIO deployment

## Vendor Dependencies
- WPILib 2025.3.2
- CTRE Phoenix 6 (latest)
- AdvantageKit (for logging)
- PathPlanner (path following)
- PhotonLib (vision processing)
- Maple-sim (simulation utilities)

## Testing Approach
- JUnit 5 for unit tests
- Test files located alongside source in standard Maven structure
- Run with `./gradlew test`

## Simulation Notes
- Simulation GUI enabled by default
- Supports AdvantageKit log replay
- Simulation implementations available for all major subsystems