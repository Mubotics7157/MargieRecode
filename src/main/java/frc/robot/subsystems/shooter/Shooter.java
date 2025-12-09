package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double targetVelocityRPM = 0.0;
    private double targetHoodAngleDegrees = 0.0;
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
            io.setHoodPosition(targetHoodAngleDegrees);
        } else {
            io.stop();
            io.stopHood();
        }

        // Log shooter state
        Logger.recordOutput("Shooter/TargetVelocityRPM", targetVelocityRPM);
        Logger.recordOutput("Shooter/TargetHoodAngle", targetHoodAngleDegrees);
        Logger.recordOutput("Shooter/Enabled", shooterEnabled);
        Logger.recordOutput("Shooter/AtSetpoint", atSetpoint());
        Logger.recordOutput("Shooter/HoodAtSetpoint", isHoodAtSetpoint());
    }

    public void setTargetVelocity(double velocityRPM) {
        this.targetVelocityRPM = velocityRPM;
    }

    public double getTargetVelocity() {
        return targetVelocityRPM;
    }

    public double getTargetHoodAngle() {
        return targetHoodAngleDegrees;
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

    // Hood control methods
    public void setHoodAngle(double degrees) {
        this.targetHoodAngleDegrees = degrees;
    }

    public double getHoodAngle() {
        return inputs.hoodPositionDegrees;
    }

    public boolean isHoodAtSetpoint() {
        return inputs.hoodAtSetpoint;
    }

    // Configure shooter for different shots
    public void configureShot(double velocityRPM, double hoodDegrees) {
        setTargetVelocity(velocityRPM);
        setHoodAngle(hoodDegrees);
    }

    // Preset shot configurations
    public void configureSpeakerShot() {
        configureShot(4000, 35.0); // 4000 RPM, 35 degree hood angle
    }

    public void configureAmpShot() {
        configureShot(1500, 60.0); // 1500 RPM, 60 degree hood angle
    }

    public void configureLongShot() {
        configureShot(5500, 25.0); // 5500 RPM, 25 degree hood angle
    }

    // Check if both rollers and hood are at setpoint
    public boolean isReadyToShoot() {
        return atSetpoint() && isHoodAtSetpoint();
    }

    // Individual roller RPM getters for dashboard
    public double getTopLeftRPM() {
        return inputs.topLeftVelocityRPM;
    }

    public double getTopRightRPM() {
        return inputs.topRightVelocityRPM;
    }

    public double getBottomLeftRPM() {
        return inputs.bottomLeftVelocityRPM;
    }

    public double getBottomRightRPM() {
        return inputs.bottomRightVelocityRPM;
    }
}
