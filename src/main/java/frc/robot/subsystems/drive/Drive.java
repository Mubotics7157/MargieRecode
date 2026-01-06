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

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LocalADStarAK;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Drive subsystem using Team 254's architecture - CTRE SwerveDrivetrain handles all module control and optimization
 * internally via SwerveRequest.
 */
public class Drive extends SubsystemBase implements Vision.VisionConsumer {

    // Drive base radius calculated from module positions
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 39.0;
    private static final double ROBOT_MOI = 3.255;
    private static final double WHEEL_COF = 1.48;

    private static final RobotConfig PP_CONFIG = new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                    TunerConstants.FrontLeft.WheelRadius,
                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                    WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                    TunerConstants.FrontLeft.SlipCurrent,
                    1),
            getModuleTranslations());

    // IO and inputs
    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    // Alerts
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    // SysId routine
    private final SysIdRoutine sysId;

    // Swerve requests (CTRE handles optimization internally)
    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withDesaturateWheelSpeeds(true)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);

    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withDesaturateWheelSpeeds(true);

    private final SwerveRequest.ApplyRobotSpeeds applySpeedsRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withDesaturateWheelSpeeds(true);

    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.SysIdSwerveTranslation sysIdTranslationRequest =
            new SwerveRequest.SysIdSwerveTranslation();

    public Drive(DriveIO io) {
        this.io = io;

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocity,
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                activePath -> Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0])));
        PathPlannerLogging.setLogTargetPoseCallback(
                targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        voltage -> io.setControl(sysIdTranslationRequest.withVolts(voltage)), null, this));
    }

    @Override
    public void periodic() {
        // Update inputs from IO
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        // Log module data
        io.logModules(null);

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            io.setControl(brakeRequest);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!inputs.gyroConnected && Constants.currentMode != Mode.SIM);
    }

    /** Runs the drive at the desired velocity using field-centric control. */
    public void runVelocityFieldCentric(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {
        io.setControl(fieldCentricRequest
                .withVelocityX(vxMetersPerSec)
                .withVelocityY(vyMetersPerSec)
                .withRotationalRate(omegaRadPerSec));
    }

    /** Runs the drive at the desired velocity using robot-centric control. */
    public void runVelocity(ChassisSpeeds speeds) {
        io.setControl(applySpeedsRequest.withSpeeds(speeds));
    }

    /** Runs the drive in a straight line with the specified drive output (for characterization). */
    public void runCharacterization(double output) {
        io.setControl(sysIdTranslationRequest.withVolts(Volts.of(output)));
    }

    /** Stops the drive. */
    public void stop() {
        io.setControl(brakeRequest);
    }

    /** Stops the drive and turns the modules to an X arrangement to resist movement. */
    public void stopWithX() {
        io.setControl(brakeRequest);
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /** Returns the current module states. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        return inputs.moduleStates;
    }

    /** Returns the measured chassis speeds. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return inputs.speeds;
    }

    /** Returns the position of each module in radians (for characterization). */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = inputs.modulePositions[i].distanceMeters / TunerConstants.FrontLeft.WheelRadius;
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (for characterization). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += inputs.moduleStates[i].speedMetersPerSecond / TunerConstants.FrontLeft.WheelRadius / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return io.getPose();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return io.getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        io.resetPose(pose);
    }

    /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    /** Returns a command that applies a SwerveRequest supplier. */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return io.applyRequest(requestSupplier, this);
    }

    /** Set control directly with a SwerveRequest. */
    public void setControl(SwerveRequest request) {
        io.setControl(request);
    }
}
