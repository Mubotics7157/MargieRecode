package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/** Constants for the drive subsystem. */
public final class DriveConstants {

    private DriveConstants() {}

    // Odometry frequency based on CAN bus type
    public static final double ODOMETRY_FREQUENCY =
            new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

    // Drive base radius (calculated from module positions)
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    // PathPlanner configuration constants
    public static final double ROBOT_MASS_KG = 39.0;
    public static final double ROBOT_MOI = 3.255;
    public static final double WHEEL_COF = 1.48;
    public static final double MAX_STEER_VELOCITY_RAD_PER_SEC = Units.rotationsToRadians(10);
}
