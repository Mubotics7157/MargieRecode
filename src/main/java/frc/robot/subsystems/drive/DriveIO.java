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

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/**
 * DriveIO interface following Team 254's architecture - wraps CTRE SwerveDrivetrain directly instead of manually
 * controlling individual modules.
 */
public interface DriveIO {

    @AutoLog
    class DriveIOInputs {
        public boolean gyroConnected = true;
        public Rotation2d gyroYaw = new Rotation2d();
        public double gyroYawVelocityRadPerSec = 0.0;

        public Pose2d pose = new Pose2d();
        public ChassisSpeeds speeds = new ChassisSpeeds();

        public SwerveModuleState[] moduleStates = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
        };

        public SwerveModuleState[] moduleTargets = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()
        };

        public SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

        public int successfulDaqs = 0;
        public int failedDaqs = 0;
        public double odometryPeriod = 0.0;

        public double[] odometryTimestamps = new double[] {};
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(DriveIOInputs inputs) {}

    /** Apply a SwerveRequest to the drivetrain. */
    default void setControl(SwerveRequest request) {}

    /** Creates a command that applies a SwerveRequest. */
    default Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystem) {
        return null;
    }

    /** Reset the pose of the drivetrain. */
    default void resetPose(Pose2d pose) {}

    /** Add a vision measurement to the pose estimator. */
    default void addVisionMeasurement(Pose2d visionPose, double timestamp) {}

    /** Add a vision measurement with standard deviations. */
    default void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

    /** Set the standard deviations for state estimation. */
    default void setStateStdDevs(double x, double y, double theta) {}

    /** Get the current pose from the drivetrain. */
    default Pose2d getPose() {
        return new Pose2d();
    }

    /** Get the current rotation from the drivetrain. */
    default Rotation2d getRotation() {
        return new Rotation2d();
    }

    /** Get the current chassis speeds. */
    default ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds();
    }

    /** Log module telemetry. */
    default void logModules(SwerveDriveState driveState) {}
}
