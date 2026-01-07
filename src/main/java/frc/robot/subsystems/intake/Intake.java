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
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // Mechanism2d for visualization
    private final Mechanism2d mechanism;
    private final MechanismLigament2d armLigament;
    private final MechanismLigament2d rollerLigament;
    private final MechanismLigament2d indexerLigament;
    private final MechanismLigament2d ballIndicator;

    // Animation state
    private double rollerSpinAngle = 0.0;
    private double indexerSpinAngle = 0.0;

    public Intake(IntakeIO io) {
        this.io = io;

        // Create Mechanism2d (width, height in meters)
        mechanism = new Mechanism2d(3, 3);

        // Create root at robot center (x, y in meters)
        MechanismRoot2d root = mechanism.getRoot("IntakeRoot", 1.5, 1.5);

        // Create arm ligament (name, length in meters, angle in degrees, line width, color)
        armLigament = root.append(new MechanismLigament2d(
                "Arm",
                0.5, // 0.5 meter arm length
                Math.toDegrees(IntakeConstants.ARM_STOWED_POSITION),
                6,
                new Color8Bit(Color.kOrange)));

        // Create roller at end of arm (animated spinner)
        rollerLigament = armLigament.append(new MechanismLigament2d(
                "Roller",
                0.15, // 0.15 meter roller visualization
                90, // Perpendicular to arm
                6,
                new Color8Bit(Color.kGreen)));

        // Create indexer visualization (separate from arm, in robot body)
        MechanismRoot2d indexerRoot = mechanism.getRoot("IndexerRoot", 1.5, 1.2);
        indexerLigament =
                indexerRoot.append(new MechanismLigament2d("Indexer", 0.2, 0, 5, new Color8Bit(Color.kYellow)));

        // Ball indicator - shows when ball is detected
        MechanismRoot2d ballRoot = mechanism.getRoot("BallRoot", 1.5, 1.0);
        ballIndicator = ballRoot.append(new MechanismLigament2d(
                "Ball",
                0.1,
                0,
                12, // Thicker line to look like a ball
                new Color8Bit(Color.kGray))); // Gray when no ball

        SmartDashboard.putData("Intake/Mechanism", mechanism);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Update mechanism visualization
        updateMechanism();
    }

    private void updateMechanism() {
        // Update arm angle
        armLigament.setAngle(Math.toDegrees(inputs.armPosition));

        // Animate roller based on velocity
        double rollerRotationRate = inputs.rollerVelocity * 0.02 * (180.0 / Math.PI); // rad/s to deg per 20ms
        rollerSpinAngle += rollerRotationRate;
        rollerSpinAngle %= 360;
        rollerLigament.setAngle(90 + rollerSpinAngle); // 90 is perpendicular to arm

        // Animate indexer
        double indexerRotationRate = inputs.indexerVelocity * 0.02 * (180.0 / Math.PI);
        indexerSpinAngle += indexerRotationRate;
        indexerSpinAngle %= 360;
        indexerLigament.setAngle(indexerSpinAngle);

        // Update roller color based on state
        if (Math.abs(inputs.rollerVelocity) > 1.0) {
            if (inputs.rollerVelocity > 0) {
                // Intaking - green
                rollerLigament.setColor(new Color8Bit(Color.kGreen));
            } else {
                // Outtaking - red
                rollerLigament.setColor(new Color8Bit(Color.kRed));
            }
        } else {
            // Stopped - gray
            rollerLigament.setColor(new Color8Bit(Color.kGray));
        }

        // Update indexer color based on state
        if (Math.abs(inputs.indexerVelocity) > 1.0) {
            if (inputs.indexerVelocity > 0) {
                // Feeding forward - yellow
                indexerLigament.setColor(new Color8Bit(Color.kYellow));
            } else {
                // Reversing - orange
                indexerLigament.setColor(new Color8Bit(Color.kOrange));
            }
        } else {
            // Stopped - gray
            indexerLigament.setColor(new Color8Bit(Color.kGray));
        }

        // Update ball indicator
        if (inputs.ballDetected) {
            // Ball detected - bright orange (game piece color)
            ballIndicator.setColor(new Color8Bit(Color.kOrangeRed));
            ballIndicator.setLength(0.15); // Larger when ball present
        } else {
            // No ball - dim gray
            ballIndicator.setColor(new Color8Bit(Color.kDarkGray));
            ballIndicator.setLength(0.05); // Smaller when no ball
        }

        // Update arm color based on position
        double armError = Math.abs(inputs.armPosition - IntakeConstants.ARM_STOWED_POSITION);
        double deployedError = Math.abs(inputs.armPosition - IntakeConstants.ARM_DEPLOYED_POSITION);

        if (armError < 0.5) {
            // At stowed position - blue
            armLigament.setColor(new Color8Bit(Color.kBlue));
        } else if (deployedError < 0.5) {
            // At deployed position - green
            armLigament.setColor(new Color8Bit(Color.kGreen));
        } else {
            // Moving - orange
            armLigament.setColor(new Color8Bit(Color.kOrange));
        }
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
        io.setArmPosition(IntakeConstants.ARM_DEPLOYED_POSITION);
    }

    // Stow the intake arm to stowed position
    public void stowArm() {
        io.setArmPosition(IntakeConstants.ARM_STOWED_POSITION);
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
        io.stopIndexer();
    }

    // Stop arm
    public void stopArm() {
        io.stopArmMotor();
    }

    // Indexer control methods
    public void setIndexerDutyCycle(double value) {
        io.setIndexerDutyCycle(value);
    }

    public void stopIndexer() {
        io.stopIndexer();
    }

    // Ball detection
    public boolean isBallDetected() {
        return inputs.ballDetected;
    }

    // Feed ball to shooter
    public void feedToShooter() {
        io.setIndexerDutyCycle(0.8); // High power to feed
    }
}
