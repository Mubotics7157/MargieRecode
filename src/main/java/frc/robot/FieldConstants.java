package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Field constants for FRC 2024 Crescendo field. Contains target positions and field dimensions used for aiming
 * calculations.
 */
public final class FieldConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.223);
    public static final double FIELD_WIDTH = Units.inchesToMeters(323.277);

    /** Blue alliance speaker target position (center of the speaker opening). */
    public static final Translation3d BLUE_SPEAKER =
            new Translation3d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), Units.inchesToMeters(80.515));

    /** Red alliance speaker target position (center of the speaker opening). */
    public static final Translation3d RED_SPEAKER = new Translation3d(
            FIELD_LENGTH + Units.inchesToMeters(1.5), Units.inchesToMeters(218.42), Units.inchesToMeters(80.515));

    /** Blue alliance speaker pose for aiming (2D projection). */
    public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(BLUE_SPEAKER.toTranslation2d(), new Rotation2d());

    /** Red alliance speaker pose for aiming (2D projection). */
    public static final Pose2d RED_SPEAKER_POSE = new Pose2d(RED_SPEAKER.toTranslation2d(), new Rotation2d());

    private FieldConstants() {}

    /** Returns the speaker position for the given alliance color. */
    public static Translation2d getSpeakerPosition(boolean isRedAlliance) {
        return isRedAlliance ? RED_SPEAKER.toTranslation2d() : BLUE_SPEAKER.toTranslation2d();
    }

    /** Returns the speaker 3D position for the given alliance color. */
    public static Translation3d getSpeaker3d(boolean isRedAlliance) {
        return isRedAlliance ? RED_SPEAKER : BLUE_SPEAKER;
    }
}
