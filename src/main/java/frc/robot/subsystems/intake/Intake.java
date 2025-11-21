package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    // Arm position constants (radians)
    public static final double ARM_STOWED_POSITION = 0.0;
    public static final double ARM_DEPLOYED_POSITION = Math.PI / 2; // 90 degrees

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }
    
    //Set roller voltage (-12 to +12 volts)
    public void setRollerVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }
    
    //Set arm voltage (-12 to +12 volts) for manual control
    public void setArmVoltage(double voltage) {
        io.setArmVoltage(voltage);
    }
    
    //Set arm to a specific position in radians
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
    
    // Check if a game piece is detected
    public boolean hasGamePiece() {
        return inputs.hasGamePiece;
    }
    
    // Get roller position in radians
    public double getRollerPositionRad() {
        return inputs.rollerPosition;
    }
    
    //Get roller velocity in rad/s
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
        io.setArmVoltage(0.0);
    }
}
