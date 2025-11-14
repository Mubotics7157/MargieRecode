package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
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
    
    /** Set roller voltage (-12 to +12 volts) */
    public void setRollerVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }
    
    /** Set arm voltage (-12 to +12 volts) */
    public void setArmVoltage(double voltage) {
        io.setArmVoltage(voltage);
    }
    
    /** Deploy the intake arm */
    public void deployArm() {
        io.deployArm();
    }
    
    /** Stow the intake arm */
    public void stowArm() {
        io.stowArm();
    }
    
    /** Reset arm encoder position to zero */
    public void resetArmPosition() {
        io.resetArmPosition();
    }
    
    /** Check if a game piece is detected */
    public boolean hasGamePiece() {
        return inputs.hasGamePiece;
    }
    
    /** Get roller position in radians */
    public double getRollerPositionRad() {
        return inputs.rollerPosition;
    }
    
    /** Get roller velocity in rad/s */
    public double getRollerVelocityRadPerSec() {
        return inputs.rollerVelocity;
    }
    
    /** Get arm position in radians */
    public double getArmPositionRad() {
        return inputs.armPosition;
    }
    
    /** Stop all motors */
    public void stop() {
        io.stopRoller();
        io.setArmVoltage(0.0);
    }
}