package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Field constants for FRC 2022 Rapid React field. Contains target positions and field dimensions used for aiming
 * calculations.
 */
public final class FieldConstants {
    // 2022 field dimensions: 54ft x 27ft
    public static final double FIELD_LENGTH = Units.feetToMeters(54.0);
    public static final double FIELD_WIDTH = Units.feetToMeters(27.0);

    // Hub is in the center of the field
    // Upper hub opening height is approximately 8ft 8in (2.64m) from floor
    public static final double HUB_HEIGHT = Units.inchesToMeters(31.0); // 104 in

    /** Hub target position (center of the field, at the upper hub opening height). */
    public static final Translation3d HUB = new Translation3d(
            Units.feetToMeters(10),
            Units.feetToMeters(3),
            HUB_HEIGHT); // FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0, HUB_HEIGHT

    /** Hub pose for aiming (2D projection at field center). */
    public static final Pose2d HUB_POSE = new Pose2d(HUB.toTranslation2d(), new Rotation2d());

    private FieldConstants() {}

    /** Returns the hub 2D position (center of field). */
    public static Translation2d getHubPosition() {
        return HUB.toTranslation2d();
    }

    /** Returns the hub 3D position. */
    public static Translation3d getHub3d() {
        return HUB;
    }
}
