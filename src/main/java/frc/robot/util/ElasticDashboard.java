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

    // Vision tab widgets (placeholders for camera streams)
    private GenericEntry camera0StreamEntry;
    private GenericEntry camera1StreamEntry;

    // Auto chooser
    private SendableChooser<Command> autoChooser;

    public ElasticDashboard() {
        // Create tabs
        mainTab = Shuffleboard.getTab("Main");
        visionTab = Shuffleboard.getTab("Vision");

        // Initialize field
        field = new Field2d();

        setupMainTab();
        setupVisionTab();
    }

    /** Sets up the main competition tab with essential robot information. */
    private void setupMainTab() {
        // Field widget - large visualization
        mainTab.add("Field", field)
                .withWidget(BuiltInWidgets.kField)
                .withSize(6, 4)
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

        // Swerve module visualization
        ShuffleboardLayout moduleLayout = mainTab.getLayout("Swerve Modules", BuiltInLayouts.kGrid)
                .withSize(4, 2)
                .withPosition(6, 3)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 2));
    }

    /** Sets up the vision tab with camera streams. */
    private void setupVisionTab() {
        // Camera 0 stream
        camera0StreamEntry = visionTab
                .add("Camera 0", "mjpg:http://limelight-camera0.local:5800")
                .withWidget(BuiltInWidgets.kCameraStream)
                .withSize(5, 4)
                .withPosition(0, 0)
                .getEntry();

        // Camera 1 stream
        camera1StreamEntry = visionTab
                .add("Camera 1", "mjpg:http://limelight-camera1.local:5800")
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

    // Updates all dashboard values
    public void update(Drive drive, Superstructure superstructure) {
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
    }

    /** Updates superstructure state information. */
    private void updateSuperstructureState(Superstructure superstructure) {}

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
}
