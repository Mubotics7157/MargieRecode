package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Detects wheel skidding on a swerve drive by exploiting measurement redundancy.
 *
 * <p>A swerve drive has 4 modules reporting 8 measurements (speed + angle each), but robot motion only has 3 degrees of
 * freedom (vx, vy, omega). This overdetermined system allows internal consistency checking.
 *
 * <p>The algorithm:
 *
 * <ol>
 *   <li>Use forward kinematics to estimate chassis state from all module measurements
 *   <li>Calculate what each module's rotational velocity component should be
 *   <li>Subtract rotation from measured to isolate the translational component
 *   <li>On a rigid body, all points share the same translational velocity
 *   <li>If extracted translations disagree, a wheel is reporting inconsistent data (skidding)
 * </ol>
 *
 * <p>The skid ratio (max translation / min translation) quantifies disagreement:
 *
 * <ul>
 *   <li>1.0 = perfect agreement, no skidding detected
 *   <li>&gt;1.0 = inconsistency detected, possible skidding
 * </ul>
 */
public class SkidDetector {
    private static final double MIN_VELOCITY_THRESHOLD = 1e-4; // m/s
    private static final int NUM_MODULES = 4;
    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

    private final SwerveDriveKinematics kinematics;
    private final Supplier<SwerveModuleState[]> moduleStatesSupplier;

    private double skidRatio = 1.0;
    private final double[] translationalMagnitudes = new double[NUM_MODULES];
    private final Translation2d[] extractedTranslations = new Translation2d[NUM_MODULES];

    /**
     * Creates a SkidDetector that pulls module states from the provided supplier.
     *
     * @param moduleStatesSupplier Supplier that returns current module states (e.g., drive::getModuleStates)
     */
    public SkidDetector(Supplier<SwerveModuleState[]> moduleStatesSupplier) {
        this.moduleStatesSupplier = moduleStatesSupplier;

        // Initialize kinematics from TunerConstants
        this.kinematics = new SwerveDriveKinematics(
                new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
                new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY));

        // Initialize translation arrays
        for (int i = 0; i < NUM_MODULES; i++) {
            extractedTranslations[i] = new Translation2d();
        }
    }

    /**
     * Updates skid detection and logs all data. Call this method periodically.
     *
     * <p>This method fetches module states from the supplier, calculates the skid ratio, and logs all relevant data to
     * AdvantageScope.
     */
    public void periodic() {
        SwerveModuleState[] moduleStates = moduleStatesSupplier.get();

        if (moduleStates == null || moduleStates.length != NUM_MODULES) {
            skidRatio = 1.0;
            log();
            return;
        }

        // Step 1: Forward kinematics - estimate chassis state from all module measurements
        // This uses least-squares regression internally to find the best fit
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(moduleStates);
        double omega = chassisSpeeds.omegaRadiansPerSecond;

        // Step 2: Inverse kinematics - calculate rotational-only velocities
        // What each module's velocity would be if the robot was only rotating (no translation)
        SwerveModuleState[] rotationalOnly = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, omega));

        // Step 3: Extract translational component for each module
        // v_trans = v_measured - v_rot
        // On a rigid body, translation is identical for all points
        double minMagnitude = Double.MAX_VALUE;
        double maxMagnitude = 0.0;

        for (int i = 0; i < NUM_MODULES; i++) {
            // Convert module states to Translation2d vectors (speed * direction)
            Translation2d measured = new Translation2d(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle);
            Translation2d rotational =
                    new Translation2d(rotationalOnly[i].speedMetersPerSecond, rotationalOnly[i].angle);

            // Subtract rotational component to isolate translation
            Translation2d translational = measured.minus(rotational);
            extractedTranslations[i] = translational;

            double magnitude = translational.getNorm();
            translationalMagnitudes[i] = magnitude;

            if (magnitude < minMagnitude) {
                minMagnitude = magnitude;
            }
            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude;
            }
        }

        // Step 4: Calculate skid ratio
        // Handle edge case: robot is nearly stationary
        if (minMagnitude < MIN_VELOCITY_THRESHOLD) {
            skidRatio = 1.0; // Can't meaningfully detect skid when not moving
        } else {
            skidRatio = maxMagnitude / minMagnitude;
        }

        log();
    }

    /**
     * Returns the current skid ratio.
     *
     * @return Skid ratio (1.0 = no skid, &gt;1.0 = potential skidding)
     */
    public double getSkidRatio() {
        return skidRatio;
    }

    /**
     * Returns whether skidding is currently detected based on threshold.
     *
     * @param threshold The skid ratio threshold above which skidding is considered detected
     * @return True if skid ratio exceeds threshold
     */
    public boolean isSkidding(double threshold) {
        return skidRatio > threshold;
    }

    /**
     * Returns the extracted translational magnitudes for each module.
     *
     * @return Array of translational velocity magnitudes (m/s)
     */
    public double[] getTranslationalMagnitudes() {
        return translationalMagnitudes;
    }

    /**
     * Returns the extracted translational velocity vectors for each module.
     *
     * @return Array of Translation2d representing each module's extracted translational velocity
     */
    public Translation2d[] getExtractedTranslations() {
        return extractedTranslations;
    }

    /** Logs all skid detection data to AdvantageScope. */
    private void log() {
        Logger.recordOutput("Drive/SkidDetector/SkidRatio", skidRatio);
        Logger.recordOutput("Drive/SkidDetector/TranslationalMagnitudes", translationalMagnitudes);

        // Log individual module translations for detailed analysis
        for (int i = 0; i < NUM_MODULES; i++) {
            String prefix = "Drive/SkidDetector/" + MODULE_NAMES[i] + "/";
            Logger.recordOutput(prefix + "TranslationalX", extractedTranslations[i].getX());
            Logger.recordOutput(prefix + "TranslationalY", extractedTranslations[i].getY());
            Logger.recordOutput(prefix + "TranslationalMagnitude", translationalMagnitudes[i]);
        }
    }
}
