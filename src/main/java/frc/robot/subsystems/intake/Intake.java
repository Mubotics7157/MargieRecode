package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    // Arm position constants (radians)
    public static final double ARM_STOWED_POSITION = 0.0;
    public static final double ARM_DEPLOYED_POSITION = -8; // -1.62

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // Mechanism2d for visualization
    private final Mechanism2d mechanism;
    private final MechanismLigament2d armLigament;
    private final MechanismLigament2d rollerLigament;

    public Intake(IntakeIO io) {
        this.io = io;

        // Create Mechanism2d (width, height in meters)
        mechanism = new Mechanism2d(3, 3);

        // Create root at robot center (x, y in meters)
        MechanismRoot2d root = mechanism.getRoot("IntakeRoot", 1.5, 0.5);

        // Create arm ligament (name, length in meters, angle in degrees, line width, color)
        armLigament = root.append(new MechanismLigament2d(
                "Arm",
                0.5, // 0.5 meter arm length
                Math.toDegrees(ARM_STOWED_POSITION),
                6,
                new Color8Bit(Color.kOrange)));

        // Create roller at end of arm
        rollerLigament = armLigament.append(new MechanismLigament2d(
                "Roller",
                0.15, // 0.15 meter roller visualization
                90, // Perpendicular to arm
                4,
                new Color8Bit(Color.kGreen)));

        SmartDashboard.putData("Intake/Mechanism", mechanism);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Update mechanism visualization
        armLigament.setAngle(Math.toDegrees(inputs.armPosition));
        rollerLigament.setColor(new Color8Bit(Color.kGreen));
    }

    // Set roller voltage (-12 to +12 volts)
    public void setRollerDutyCycle(double value) {
        io.setRollerDutyCycle(value);
    }

    // Set arm voltage (-12 to +12 volts) for manual control
    public void setArmVoltage(double voltage) {
        io.setArmVoltage(voltage);
    }

    // Set arm to a specific position in radians
    public void setArmPosition(double positionRad) {
        io.setArmPosition(positionRad);
    }

    // Deploy the intake arm to deployed position
    public void deployArm() {
        io.setArmPosition(ARM_DEPLOYED_POSITION);
    }

    // Stow the intake arm to stowed position
    public void stowArm() {
        io.setArmPosition(ARM_STOWED_POSITION);
    }

    // Reset arm encoder position to zero
    public void resetArmPosition() {
        io.resetArmPosition();
    }

    // GETTERS

    // Roller telemetry
    public double getRollerPositionRad() {
        return inputs.rollerPosition;
    }

    public double getRollerVelocityRadPerSec() {
        return inputs.rollerVelocity;
    }

    public double getRollerCurrent() {
        return inputs.rollerCurrent;
    }

    public double getRollerVoltage() {
        return inputs.rollerVoltage;
    }

    // Arm telemetry
    public double getArmPositionRad() {
        return inputs.armPosition;
    }

    public double getArmVelocityRadPerSec() {
        return inputs.armVelocity;
    }

    public double getArmCurrent() {
        return inputs.armCurrent;
    }

    public double getArmVoltage() {
        return inputs.armVoltage;
    }

    // Indexer telemetry
    public double getIndexerPositionRad() {
        return inputs.indexerPosition;
    }

    public double getIndexerVelocityRadPerSec() {
        return inputs.indexerVelocity;
    }

    public double getIndexerCurrent() {
        return inputs.indexerCurrent;
    }

    public double getIndexerVoltage() {
        return inputs.indexerVoltage;
    }

    // Check if arm is at target position (within tolerance)
    public boolean isArmAtPosition(double targetRad, double toleranceRad) {
        return Math.abs(inputs.armPosition - targetRad) < toleranceRad;
    }

    // Get the Mechanism2d for dashboard display
    public Mechanism2d getMechanism() {
        return mechanism;
    }

    // Stop all motors
    public void stop() {
        io.stopRoller();
    }

    // Stop arm
    public void stopArm() {
        io.stopArmMotor();
    }
}
