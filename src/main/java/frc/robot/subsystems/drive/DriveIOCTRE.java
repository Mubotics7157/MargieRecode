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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * DriveIOCTRE extends CTRE's SwerveDrivetrain directly, following Team 254's architecture. CTRE handles all module
 * control and optimization internally.
 */
public class DriveIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {

    // Thread-safe telemetry cache
    protected final AtomicReference<SwerveDriveState> telemetryCache = new AtomicReference<>();

    // Gyro signals
    private final StatusSignal<AngularVelocity> gyroYawVelocity;

    // Telemetry consumer that caches state
    protected Consumer<SwerveDriveState> telemetryConsumer = state -> telemetryCache.set(state.clone());

    public DriveIOCTRE(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, odometryUpdateFrequency, modules);

        // Get gyro signals
        gyroYawVelocity = getPigeon2().getAngularVelocityZWorld();
        BaseStatusSignal.setUpdateFrequencyForAll(odometryUpdateFrequency, gyroYawVelocity);

        // Set odometry thread priority
        getOdometryThread().setThreadPriority(99);

        // Register telemetry consumer
        registerTelemetry(telemetryConsumer);
    }

    public DriveIOCTRE(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        this(drivetrainConstants, 250.0, modules);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        SwerveDriveState state = telemetryCache.get();
        if (state == null) return;

        // Gyro inputs
        inputs.gyroConnected = BaseStatusSignal.refreshAll(gyroYawVelocity).isOK();
        inputs.gyroYaw = state.Pose.getRotation();
        inputs.gyroYawVelocityRadPerSec = Math.toRadians(gyroYawVelocity.getValueAsDouble());

        // Pose and speeds
        inputs.pose = state.Pose;
        inputs.speeds = state.Speeds;

        // Module states
        if (state.ModuleStates != null) {
            inputs.moduleStates = state.ModuleStates.clone();
        }
        if (state.ModuleTargets != null) {
            inputs.moduleTargets = state.ModuleTargets.clone();
        }

        // Module positions
        inputs.modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            inputs.modulePositions[i] = getModule(i).getPosition(false);
        }

        // DAQ status
        inputs.successfulDaqs = state.SuccessfulDaqs;
        inputs.failedDaqs = state.FailedDaqs;
        inputs.odometryPeriod = state.OdometryPeriod;
    }

    @Override
    public void setControl(SwerveRequest request) {
        super.setControl(request);
    }

    @Override
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystem) {
        return Commands.run(() -> setControl(requestSupplier.get()), subsystem);
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        super.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestamp));
    }

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        super.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestamp), stdDevs);
    }

    @Override
    public void setStateStdDevs(double x, double y, double theta) {
        super.setStateStdDevs(VecBuilder.fill(x, y, theta));
    }

    @Override
    public Pose2d getPose() {
        SwerveDriveState state = telemetryCache.get();
        return state != null ? state.Pose : new Pose2d();
    }

    @Override
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        SwerveDriveState state = telemetryCache.get();
        return state != null ? state.Speeds : new ChassisSpeeds();
    }

    @Override
    public void logModules(SwerveDriveState driveState) {
        if (driveState == null || driveState.ModuleStates == null) return;

        final String[] moduleNames = {"Drive/FL", "Drive/FR", "Drive/BL", "Drive/BR"};
        for (int i = 0; i < getModules().length; i++) {
            Logger.recordOutput(
                    moduleNames[i] + "/AbsoluteEncoderAngle",
                    getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble() * 360);
            Logger.recordOutput(moduleNames[i] + "/SteerAngle", driveState.ModuleStates[i].angle.getDegrees());
            Logger.recordOutput(moduleNames[i] + "/TargetSteerAngle", driveState.ModuleTargets[i].angle.getDegrees());
            Logger.recordOutput(moduleNames[i] + "/DriveVelocity", driveState.ModuleStates[i].speedMetersPerSecond);
            Logger.recordOutput(
                    moduleNames[i] + "/TargetDriveVelocity", driveState.ModuleTargets[i].speedMetersPerSecond);
        }
    }
}
