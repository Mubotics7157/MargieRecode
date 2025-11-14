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
    
    public void setRollerVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }
    
    public void deployArm() {
        io.deployArm();
    }
    
    public void stowArm() {
        io.stowArm();
    }
    
    public boolean hasGamePiece() {
        return inputs.hasGamePiece;
    }
    
    public void stop() {
        io.stopRoller();
    }
}