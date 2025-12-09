package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double targetVelocityRPM = 0.0;
    private boolean shooterEnabled = false;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Control shooter based on enable state
        if (shooterEnabled && targetVelocityRPM > 0) {
            io.setVelocity(targetVelocityRPM, 12.0); // 12V feedforward
        } else {
            io.stop();
        }

        // Log shooter state
        Logger.recordOutput("Shooter/TargetVelocityRPM", targetVelocityRPM);
        Logger.recordOutput("Shooter/Enabled", shooterEnabled);
        Logger.recordOutput("Shooter/AtSetpoint", atSetpoint());
    }

    public void setTargetVelocity(double velocityRPM) {
        this.targetVelocityRPM = velocityRPM;
    }

    public void enable() {
        shooterEnabled = true;
    }

    public void disable() {
        shooterEnabled = false;
        targetVelocityRPM = 0.0;
    }

    public boolean atSetpoint() {
        if (!shooterEnabled || targetVelocityRPM == 0) {
            return false;
        }
        double error = Math.abs(inputs.velocityRPM - targetVelocityRPM);
        return error < 100; // Within 100 RPM
    }

    public double getVelocityRPM() {
        return inputs.velocityRPM;
    }

    public boolean isEnabled() {
        return shooterEnabled;
    }
}
