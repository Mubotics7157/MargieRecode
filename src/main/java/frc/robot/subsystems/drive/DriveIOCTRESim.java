// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.MapleSimSwerveDrivetrain;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * DriveIOCTRESim extends DriveIOCTRE to provide simulation-specific functionality using Maple-Sim for physics
 * simulation. Based on Team 254's DriveIOSim.
 */
public class DriveIOCTRESim extends DriveIOCTRE {

    private static final double kSimLoopPeriod = 0.005; // 5 ms

    private MapleSimSwerveDrivetrain mapleSimDrivetrain = null;
    private Notifier simNotifier = null;

    // Configuration
    private final double robotMassKg;
    private final double bumperLengthMeters;
    private final double bumperWidthMeters;
    private final double wheelCOF;
    private final Translation2d[] moduleLocations;

    @SuppressWarnings("unchecked")
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
            moduleConstants;

    // Telemetry consumer that also updates simulation pose
    private final Consumer<SwerveDriveState> simTelemetryConsumer = state -> {
        if (mapleSimDrivetrain != null) {
            // Use the simulated pose instead of odometry
            state.Pose = mapleSimDrivetrain.getSimulatedPose();
        }
        telemetryCache.set(state.clone());
    };

    @SuppressWarnings("unchecked")
    public DriveIOCTRESim(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            double robotMassKg,
            double bumperLengthMeters,
            double bumperWidthMeters,
            double wheelCOF,
            Translation2d[] moduleLocations,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));

        this.robotMassKg = robotMassKg;
        this.bumperLengthMeters = bumperLengthMeters;
        this.bumperWidthMeters = bumperWidthMeters;
        this.wheelCOF = wheelCOF;
        this.moduleLocations = moduleLocations;

        // Store regulated module constants
        this.moduleConstants =
                (SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[])
                        MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules);

        // Override telemetry consumer for simulation
        registerTelemetry(simTelemetryConsumer);

        // Start simulation thread
        startSimThread();
    }

    @SuppressWarnings("unchecked")
    private void startSimThread() {
        mapleSimDrivetrain = new MapleSimSwerveDrivetrain(
                Seconds.of(kSimLoopPeriod),
                Kilograms.of(robotMassKg),
                Meters.of(bumperLengthMeters),
                Meters.of(bumperWidthMeters),
                DCMotor.getKrakenX60(1),
                DCMotor.getKrakenX60(1),
                wheelCOF,
                moduleLocations,
                getPigeon2(),
                getModules(),
                moduleConstants);

        simNotifier = new Notifier(mapleSimDrivetrain::update);
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (mapleSimDrivetrain != null) {
            mapleSimDrivetrain.setSimulationWorldPose(pose);
            Timer.delay(0.05); // Allow simulation to update
        }
        super.resetPose(pose);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        super.updateInputs(inputs);

        // Log simulation-specific data
        if (mapleSimDrivetrain != null) {
            Logger.recordOutput("Drive/SimulatedPose", mapleSimDrivetrain.getSimulatedPose());
        }
    }

    @Override
    public Pose2d getPose() {
        // In simulation, prefer the simulated pose
        if (mapleSimDrivetrain != null) {
            return mapleSimDrivetrain.getSimulatedPose();
        }
        return super.getPose();
    }

    /** Get the MapleSimSwerveDrivetrain instance for external access. */
    public MapleSimSwerveDrivetrain getMapleSimDrivetrain() {
        return mapleSimDrivetrain;
    }
}
