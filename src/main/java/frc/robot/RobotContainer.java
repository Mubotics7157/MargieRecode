// Copyright 2021-2024 FRC 6328
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

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.pathplanner.auto.AutoBuilder;
import frc.robot.lib.pathplanner.auto.NamedCommands;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.led.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureIO;
import frc.robot.subsystems.superstructure.SuperstructureIOReal;
import frc.robot.subsystems.vision.*;
import frc.robot.util.ElasticDashboard;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Vision vision;
    private final Intake intake;
    private final Shooter shooter;
    private final Superstructure superstructure;
    private final Led led;

    // Elastic Dashboard
    private final ElasticDashboard elasticDashboard;

    // Simulation reference (for resetting)
    private DriveIOCTRESim driveIOSim = null;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    // Robot configuration
    private static final double ROBOT_MASS_KG = 39.0;
    private static final double BUMPER_LENGTH_METERS = Units.inchesToMeters(32);
    private static final double BUMPER_WIDTH_METERS = Units.inchesToMeters(32);
    private static final double WHEEL_COF = 1.48;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot - use CTRE SwerveDrivetrain directly
                drive = new Drive(new DriveIOCTRE(
                        TunerConstants.DrivetrainConstants,
                        250.0,
                        TunerConstants.FrontLeft,
                        TunerConstants.FrontRight,
                        TunerConstants.BackLeft,
                        TunerConstants.BackRight));
                this.vision = new Vision(
                        drive,
                        new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                        new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
                intake = new Intake(new IntakeIOTalonFXReal());
                shooter = new Shooter(new ShooterIOTalonFXReal());
                superstructure = new Superstructure(intake, shooter, new SuperstructureIOReal() {});
                led = new Led(new LedIOCANdle(
                        Constants.LedConstants.kCandleId,
                        Constants.LedConstants.kCanBus,
                        Constants.LedConstants.kTotalLedCount));
                break;

            case SIM:
                // Simulation - use CTRE SwerveDrivetrain with MapleSim
                driveIOSim = new DriveIOCTRESim(
                        TunerConstants.DrivetrainConstants,
                        250.0,
                        ROBOT_MASS_KG,
                        BUMPER_LENGTH_METERS,
                        BUMPER_WIDTH_METERS,
                        WHEEL_COF,
                        Drive.getModuleTranslations(),
                        TunerConstants.FrontLeft,
                        TunerConstants.FrontRight,
                        TunerConstants.BackLeft,
                        TunerConstants.BackRight);
                drive = new Drive(driveIOSim);
                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                        new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
                intake = new Intake(new IntakeIOTalonFXSim());
                shooter = new Shooter(new ShooterIOTalonFXSim());
                superstructure = new Superstructure(intake, shooter, new SuperstructureIOReal() {});
                led = new Led(new LedIO() {}); // Simulation doesn't need real LED hardware
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(new DriveIO() {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});
                intake = new Intake(new IntakeIO() {});
                shooter = new Shooter(new ShooterIO() {});
                superstructure = new Superstructure(intake, shooter, new SuperstructureIO() {});
                led = new Led(new LedIO() {});
                break;
        }

        // Set superstructure reference for automatic LED state
        led.setSuperstructure(superstructure);

        NamedCommands.registerCommand("SetIntake", SuperstructureCommands.intake(superstructure));
        NamedCommands.registerCommand("SetIdle", SuperstructureCommands.setIdle(superstructure));
        NamedCommands.registerCommand("AutoIntake", SuperstructureCommands.intakeUntilReady(superstructure));

        // Initialize RobotState with pose supplier for aiming calculations
        RobotState.getInstance().setPoseSupplier(drive::getPose);

        // Initialize Elastic Dashboard
        elasticDashboard = new ElasticDashboard();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();

        // Configure camera streams (adjust URLs if needed)
        if (Constants.currentMode == Constants.Mode.REAL) {
            elasticDashboard.setCameraStreams(
                    "mjpg:http://" + camera0Name + ".local:5800", "mjpg:http://" + camera1Name + ".local:5800");
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // ============DRIVER CONTROLLER BINDINGS (DRIVE)============//
        // Default command, field-relative drive with heading maintenance
        // Based on Team 254's DriveMaintainingHeadingCommand
        drive.setDefaultCommand(DriveCommands.joystickDriveMaintainHeading(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));

        // Reset gyro / odometry
        final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
                ? () -> drive.setPose(drive.getPose()) // In sim, just reset to current pose
                : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
        controller.povDown().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        // ============DRIVER CONTROLLER BINDINGS (SYSTEM)============//
        // Intake
        controller.leftBumper().whileTrue(SuperstructureCommands.intake(superstructure)); // left bumper

        // Eject
        controller.rightBumper().whileTrue(SuperstructureCommands.outtake(superstructure)); // right bumper

        // Aim and Shoot - automatically rotates robot to face hub while shooting
        DoubleSupplier xInput = () -> -controller.getLeftY();
        DoubleSupplier yInput = () -> -controller.getLeftX();
        controller
                .rightTrigger()
                .whileTrue(SuperstructureCommands.aimAndShoot(
                        drive, superstructure, shooter, xInput, yInput)); // right trigger

        // Aim only (pre-aim without shooting) - left trigger
        controller.leftTrigger().whileTrue(SuperstructureCommands.aimAtHub(drive, xInput, yInput)); // left trigger
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void resetSimulationField() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        if (driveIOSim != null && driveIOSim.getMapleSimDrivetrain() != null) {
            driveIOSim.getMapleSimDrivetrain().setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        }
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void updateSimulation() {
        if (Constants.currentMode != Constants.Mode.SIM) return;

        // MapleSim updates are handled by the Notifier in DriveIOCTRESim
        // Just log the simulation state here
        if (driveIOSim != null && driveIOSim.getMapleSimDrivetrain() != null) {
            Logger.recordOutput(
                    "FieldSimulation/RobotPosition",
                    driveIOSim.getMapleSimDrivetrain().getSimulatedPose());
        }
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }

    public void updateDashboard() {
        elasticDashboard.update(drive, superstructure, intake, shooter);
    }

    public ElasticDashboard getElasticDashboard() {
        return elasticDashboard;
    }
}
