package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.superstructure.Superstructure;
import java.util.Map;

/**
 * Utility class for managing Elastic Dashboard layouts and widgets. Provides a centralized interface for displaying
 * robot telemetry.
 */
public class ElasticDashboard {
    // Tabs
    private final ShuffleboardTab mainTab;
    private final ShuffleboardTab visionTab;
    private final ShuffleboardTab mechanismTab;

    // Field widget
    private final Field2d field;

    // Main tab widgets
    private GenericEntry currentStateEntry;
    private GenericEntry desiredStateEntry;
    private GenericEntry atGoalEntry;
    private GenericEntry matchTimeEntry;
    private GenericEntry batteryVoltageEntry;
    private GenericEntry allianceEntry;

    // Drive tab widgets
    private GenericEntry driveXEntry;
    private GenericEntry driveYEntry;
    private GenericEntry driveRotationEntry;
    private GenericEntry gyroYawEntry;

    // Mechanism tab widgets - Intake
    private GenericEntry armCurrentEntry;
    private GenericEntry armVoltageEntry;
    private GenericEntry armPositionEntry;
    private GenericEntry armVelocityEntry;
    private GenericEntry rollerCurrentEntry;
    private GenericEntry rollerVoltageEntry;
    private GenericEntry rollerPositionEntry;
    private GenericEntry rollerVelocityEntry;
    private GenericEntry indexerCurrentEntry;
    private GenericEntry indexerVoltageEntry;
    private GenericEntry indexerPositionEntry;
    private GenericEntry indexerVelocityEntry;
    private GenericEntry ballDetectedEntry;

    // Mechanism tab widgets - Shooter
    private GenericEntry shooterVelocityEntry;
    private GenericEntry shooterTargetVelocityEntry;
    private GenericEntry shooterAtSetpointEntry;
    private GenericEntry shooterEnabledEntry;
    private GenericEntry hoodPositionEntry;
    private GenericEntry hoodTargetPositionEntry;
    private GenericEntry hoodAtSetpointEntry;
    private GenericEntry flywheelMidRPMEntry;
    private GenericEntry flywheelRightRPMEntry;
    private GenericEntry flywheel2InRPMEntry;
    private GenericEntry shooterMotorRPMEntry;
    private GenericEntry pooperRPMEntry;

    // Vision tab widgets (placeholders for camera streams)
    private GenericEntry camera0StreamEntry;
    private GenericEntry camera1StreamEntry;

    // Auto chooser
    private SendableChooser<Command> autoChooser;

    public ElasticDashboard() {
        // Create tabs
        mainTab = Shuffleboard.getTab("Main");
        visionTab = Shuffleboard.getTab("Vision");
        mechanismTab = Shuffleboard.getTab("Mechanisms");

        // Initialize field
        field = new Field2d();

        setupMainTab();
        setupVisionTab();
        setupMechTab();
    }

    /** Sets up the main competition tab with essential robot information. */
    private void setupMainTab() {
        // Field widget - large visualization
        mainTab.add("Field", field)
                .withWidget(BuiltInWidgets.kField)
                .withSize(6, 3)
                .withPosition(0, 0);

        // Robot Status layout
        ShuffleboardLayout robotStatus = mainTab.getLayout("Robot Status", BuiltInLayouts.kList)
                .withSize(2, 3)
                .withPosition(6, 0);

        matchTimeEntry = robotStatus
                .add("Match Time", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        batteryVoltageEntry = robotStatus
                .add("Battery Voltage", 0.0)
                .withWidget(BuiltInWidgets.kVoltageView)
                .withProperties(Map.of("min", 0, "max", 13))
                .getEntry();

        allianceEntry = robotStatus
                .add("Alliance", "Unknown")
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        // Superstructure Status layout
        ShuffleboardLayout superstructureStatus = mainTab.getLayout("Superstructure", BuiltInLayouts.kList)
                .withSize(2, 3)
                .withPosition(8, 0);

        currentStateEntry = superstructureStatus
                .add("Current State", "IDLE")
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        desiredStateEntry = superstructureStatus
                .add("Desired State", "IDLE")
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        atGoalEntry = superstructureStatus
                .add("At Goal", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"))
                .getEntry();

        // Drive Telemetry layout
        ShuffleboardLayout driveTelemetry = mainTab.getLayout("Drive Telemetry", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(10, 0);

        driveXEntry = driveTelemetry
                .add("X Position (m)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        driveYEntry = driveTelemetry
                .add("Y Position (m)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        driveRotationEntry = driveTelemetry
                .add("Rotation (deg)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        gyroYawEntry = driveTelemetry
                .add("Gyro Yaw (deg)", 0.0)
                .withWidget(BuiltInWidgets.kGyro)
                .getEntry();

        // Intake value summary
        ShuffleboardLayout intakeSummary = mainTab.getLayout("Intake Summary", BuiltInLayouts.kList)
                .withSize(2, 3)
                .withPosition(6, 3);

        intakeSummary
                .add("Arm Position", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", -10, "max", 1));

        intakeSummary
                .add("Roller Speed", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", -100, "max", 100));

        intakeSummary
                .add("Ball Detected", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "gray"));

        // Shooter Status on main tab
        ShuffleboardLayout shooterStatus = mainTab.getLayout("Shooter Status", BuiltInLayouts.kList)
                .withSize(2, 3)
                .withPosition(0, 3);

        shooterStatus
                .add("Ready to Shoot", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"));

        shooterStatus
                .add("Shooter RPM", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 0, "max", 6000));

        shooterStatus
                .add("Hood Angle", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 0, "max", 45));

        // Swerve module visualization
        ShuffleboardLayout moduleLayout = mainTab.getLayout("Swerve Modules", BuiltInLayouts.kGrid)
                .withSize(4, 2)
                .withPosition(10, 3)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 2));
    }

    // Sets up the vision tab with camera streams
    private void setupVisionTab() {
        // Camera 0 stream
        camera0StreamEntry = visionTab
                .add("Camera 0", "mjpg:http://limelight-dihlite.local:5800")
                .withWidget(BuiltInWidgets.kCameraStream)
                .withSize(5, 4)
                .withPosition(0, 0)
                .getEntry();

        // Camera 1 stream
        camera1StreamEntry = visionTab
                .add("Camera 1", "mjpg:http://limelight-dihlite.local:5800")
                .withWidget(BuiltInWidgets.kCameraStream)
                .withSize(5, 4)
                .withPosition(5, 0)
                .getEntry();

        // Vision status
        ShuffleboardLayout visionStatus = visionTab
                .getLayout("Vision Status", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(10, 0);
    }

    // Sets the auto chooser for the dashboard. Should be called from RobotContainer after AutoBuilder is configured.
    public void setAutoChooser(SendableChooser<Command> chooser) {
        this.autoChooser = chooser;
        mainTab.add("Auto Chooser", chooser)
                .withWidget(BuiltInWidgets.kComboBoxChooser)
                .withSize(2, 1)
                .withPosition(0, 4);
    }

    private void setupMechTab() {
        ShuffleboardLayout IntakeArmValues = mechanismTab
                .getLayout("Intake Arm", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0);

        armCurrentEntry = IntakeArmValues.add("Arm Current", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        armVoltageEntry = IntakeArmValues.add("Arm Voltage", 0.0)
                .withWidget(BuiltInWidgets.kVoltageView)
                .getEntry();

        armPositionEntry = IntakeArmValues.add("Arm Position (Rad)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        armVelocityEntry = IntakeArmValues.add("Arm Velocity (Rad/s)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        ShuffleboardLayout IntakeRollerValues = mechanismTab
                .getLayout("Intake Roller", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0);

        rollerCurrentEntry = IntakeRollerValues.add("Roller Current", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        rollerVoltageEntry = IntakeRollerValues.add("Roller Voltage", 0.0)
                .withWidget(BuiltInWidgets.kVoltageView)
                .getEntry();

        rollerPositionEntry = IntakeRollerValues.add("Roller Position (Rad)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        rollerVelocityEntry = IntakeRollerValues.add("Roller Velocity (Rad/s)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        ShuffleboardLayout IndexerValues = mechanismTab
                .getLayout("Indexer", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0);

        indexerCurrentEntry = IndexerValues.add("Indexer Current", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        indexerVoltageEntry = IndexerValues.add("Indexer Voltage", 0.0)
                .withWidget(BuiltInWidgets.kVoltageView)
                .getEntry();

        indexerPositionEntry = IndexerValues.add("Indexer Position (Rad)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        indexerVelocityEntry = IndexerValues.add("Indexer Velocity (Rad/s)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        ballDetectedEntry = IndexerValues.add("Ball Detected", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "gray"))
                .getEntry();

        // Shooter Main Values
        ShuffleboardLayout ShooterValues = mechanismTab
                .getLayout("Shooter", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0);

        shooterVelocityEntry = ShooterValues.add("Current Velocity (RPM)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        shooterTargetVelocityEntry = ShooterValues.add("Target Velocity (RPM)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        shooterAtSetpointEntry = ShooterValues.add("At Setpoint", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"))
                .getEntry();

        shooterEnabledEntry = ShooterValues.add("Shooter Enabled", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "gray"))
                .getEntry();

        // Shooter Hood Values
        ShuffleboardLayout HoodValues = mechanismTab
                .getLayout("Hood", BuiltInLayouts.kList)
                .withSize(2, 3)
                .withPosition(8, 0);

        hoodPositionEntry = HoodValues.add("Hood Position (deg)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        hoodTargetPositionEntry = HoodValues.add("Target Position (deg)", 0.0)
                .withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        hoodAtSetpointEntry = HoodValues.add("Hood At Setpoint", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "red"))
                .getEntry();

        // Flywheel Speeds
        ShuffleboardLayout FlywheelSpeeds = mechanismTab
                .getLayout("Flywheel Speeds", BuiltInLayouts.kList)
                .withSize(2, 3)
                .withPosition(10, 0);

        flywheelMidRPMEntry = FlywheelSpeeds.add("Flywheel Mid", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 0, "max", 6000))
                .getEntry();

        flywheelRightRPMEntry = FlywheelSpeeds.add("Flywheel Right", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 0, "max", 6000))
                .getEntry();

        flywheel2InRPMEntry = FlywheelSpeeds.add("2in Flywheel", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 0, "max", 6000))
                .getEntry();

        // Indexer Speeds
        ShuffleboardLayout IndexerSpeeds = mechanismTab
                .getLayout("Indexer Speeds", BuiltInLayouts.kList)
                .withSize(2, 2)
                .withPosition(10, 3);

        shooterMotorRPMEntry = IndexerSpeeds.add("Shooter Motor", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", -6000, "max", 6000))
                .getEntry();

        pooperRPMEntry = IndexerSpeeds.add("Pooper", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", -6000, "max", 6000))
                .getEntry();
    }

    // Updates all dashboard values
    public void update(Drive drive, Superstructure superstructure, Intake intake, Shooter shooter) {
        // Update match time
        double matchTime = DriverStation.getMatchTime();
        matchTimeEntry.setDouble(matchTime);

        // Update battery voltage
        double batteryVoltage = RobotController.getBatteryVoltage();
        batteryVoltageEntry.setDouble(batteryVoltage);

        // Update alliance
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            allianceEntry.setString(alliance.get().toString());
        } else {
            allianceEntry.setString("Unknown");
        }

        // Update superstructure state
        updateSuperstructureState(superstructure);

        // Update drive telemetry
        updateDriveTelemetry(drive);

        // Update field
        updateField(drive);

        // Update Mechanism values
        updateMechs(intake);

        // Update Shooter values
        updateShooter(shooter);
    }

    // Updates superstructure state info
    private void updateSuperstructureState(Superstructure superstructure) {
        atGoalEntry.setBoolean(superstructure.atGoal());
        currentStateEntry.setString(superstructure.getCurrentGoal().toString());
        desiredStateEntry.setString(superstructure.getDesiredGoal().toString());
    }

    private void updateMechs(Intake intake) {

        // Update Intake Arm values
        armCurrentEntry.setDouble(intake.getArmCurrent());
        armVoltageEntry.setDouble(intake.getArmVoltage());
        armPositionEntry.setDouble(intake.getArmPositionRad());
        armVelocityEntry.setDouble(intake.getArmVelocityRadPerSec());

        // Update Intake Roller values
        rollerCurrentEntry.setDouble(intake.getRollerCurrent());
        rollerVoltageEntry.setDouble(intake.getRollerVoltage());
        rollerPositionEntry.setDouble(intake.getRollerPositionRad());
        rollerVelocityEntry.setDouble(intake.getRollerVelocityRadPerSec());

        // Update Indexer values
        indexerCurrentEntry.setDouble(intake.getIndexerCurrent());
        indexerVoltageEntry.setDouble(intake.getIndexerVoltage());
        indexerPositionEntry.setDouble(intake.getIndexerPositionRad());
        indexerVelocityEntry.setDouble(intake.getIndexerVelocityRadPerSec());

        // Update ball detection
        ballDetectedEntry.setBoolean(intake.isBallDetected());
    }

    // Updates drive info
    private void updateDriveTelemetry(Drive drive) {
        Pose2d pose = drive.getPose();
        Rotation2d rotation = drive.getRotation();

        // Update position
        driveXEntry.setDouble(pose.getX());
        driveYEntry.setDouble(pose.getY());
        driveRotationEntry.setDouble(rotation.getDegrees());
        gyroYawEntry.setDouble(rotation.getDegrees());
    }

    // Updates the field widget with current robot pose
    private void updateField(Drive drive) {
        field.setRobotPose(drive.getPose());
    }

    // gets field for external use
    public Field2d getField() {
        return field;
    }

    // Set camera stream URLS
    public void setCameraStreams(String camera0Url, String camera1Url) {
        camera0StreamEntry.setString(camera0Url);
        camera1StreamEntry.setString(camera1Url);
    }

    // Updates shooter info
    private void updateShooter(Shooter shooter) {
        // Get the shooter inputs through the public methods
        shooterVelocityEntry.setDouble(shooter.getFlywheelRPM());
        shooterTargetVelocityEntry.setDouble(shooter.getTargetFlywheelRPM());
        shooterAtSetpointEntry.setBoolean(shooter.flywheelAtSetpoint());
        shooterEnabledEntry.setBoolean(shooter.isEnabled());

        // Hood values
        hoodPositionEntry.setDouble(shooter.getHoodAngle());
        hoodTargetPositionEntry.setDouble(shooter.getTargetHoodPosition());
        hoodAtSetpointEntry.setBoolean(shooter.isHoodAtSetpoint());

        // Individual flywheel speeds
        flywheelMidRPMEntry.setDouble(shooter.getFlywheelMidRPM());
        flywheelRightRPMEntry.setDouble(shooter.getFlywheelRightRPM());
        flywheel2InRPMEntry.setDouble(shooter.getFlywheel2InRPM());

        // Indexer speeds
        shooterMotorRPMEntry.setDouble(shooter.getShooterMotorRPM());
        pooperRPMEntry.setDouble(shooter.getPooperRPM());
    }
}
