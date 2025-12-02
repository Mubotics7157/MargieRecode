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

    // Get roller position in radians
    public double getRollerPositionRad() {
        return inputs.rollerPosition;
    }

    // Get roller velocity in rad/s
    public double getRollerVelocityRadPerSec() {
        return inputs.rollerVelocity;
    }

    // Get arm position in radians
    public double getArmPositionRad() {
        return inputs.armPosition;
    }

    // Get arm velocity in rad/s
    public double getArmVelocityRadPerSec() {
        return inputs.armVelocity;
    }

    // Check if arm is at target position (within tolerance)
    public boolean isArmAtPosition(double targetRad, double toleranceRad) {
        return Math.abs(inputs.armPosition - targetRad) < toleranceRad;
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
