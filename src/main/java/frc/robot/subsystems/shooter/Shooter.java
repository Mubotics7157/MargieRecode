package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double targetFlywheelRPM = 0.0;
    private double targetFlywheel2InRPM = 0.0;
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
        if (shooterEnabled && targetFlywheelRPM > 0) {
            io.setFlywheelVelocity(targetFlywheelRPM, 12.0); // 12V feedforward
            io.setFlywheel2InVelocity(targetFlywheel2InRPM, 12.0);
            io.setHoodPosition(targetHoodAngleDegrees);
        } else if (!shooterEnabled) {
            io.stopFlywheels();
            io.stopHood();
        }

        // Log shooter state
        Logger.recordOutput("Shooter/TargetFlywheelRPM", targetFlywheelRPM);
        Logger.recordOutput("Shooter/TargetFlywheel2InRPM", targetFlywheel2InRPM);
        Logger.recordOutput("Shooter/TargetHoodAngle", targetHoodAngleDegrees);
        Logger.recordOutput("Shooter/Enabled", shooterEnabled);
        Logger.recordOutput("Shooter/FlywheelAtSetpoint", flywheelAtSetpoint());
        Logger.recordOutput("Shooter/HoodAtSetpoint", isHoodAtSetpoint());
    }

    // Flywheel control
    public void setTargetFlywheelRPM(double velocityRPM) {
        this.targetFlywheelRPM = velocityRPM;
    }

    public void setTargetFlywheel2InRPM(double velocityRPM) {
        this.targetFlywheel2InRPM = velocityRPM;
    }

    public double getTargetFlywheelRPM() {
        return targetFlywheelRPM;
    }

    public double getTargetHoodAngle() {
        return targetHoodAngleDegrees;
    }

    public void enable() {
        shooterEnabled = true;
    }

    public void disable() {
        shooterEnabled = false;
        targetFlywheelRPM = 0.0;
        targetFlywheel2InRPM = 0.0;
    }

    public boolean flywheelAtSetpoint() {
        if (!shooterEnabled || targetFlywheelRPM == 0) {
            return false;
        }
        double error = Math.abs(inputs.velocityRPM - targetFlywheelRPM);
        return error < 100; // Within 100 RPM
    }

    public double getFlywheelRPM() {
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

    // Indexer control (starwheels)
    public void runIndexer(double volts) {
        io.setIndexerVoltage(volts);
    }

    public void stopIndexer() {
        io.stopIndexer();
    }

    // Pooper control
    public void ejectBall() {
        // Positive voltage = eject from robot (clockwise)
        io.setPooperVoltage(12.0);
    }

    public void feedBall() {
        // Negative voltage = continue through ball path (counter-clockwise)
        io.setPooperVoltage(-12.0);
    }

    public void stopPooper() {
        io.stopPooper();
    }

    // Configure shooter for different shots
    public void configureShot(double flywheelRPM, double flywheel2InRPM, double hoodDegrees) {
        setTargetFlywheelRPM(flywheelRPM);
        setTargetFlywheel2InRPM(flywheel2InRPM);
        setHoodAngle(hoodDegrees);
    }

    // Preset shot configurations
    public void configureSpeakerShot() {
        configureShot(4000, 3000, 35.0); // Hood flywheels, 2in flywheel, hood angle
    }

    public void configureAmpShot() {
        configureShot(1500, 1200, 60.0);
    }

    public void configureLongShot() {
        configureShot(5500, 4500, 25.0);
    }

    // Check if both flywheels and hood are at setpoint
    public boolean isReadyToShoot() {
        return flywheelAtSetpoint() && isHoodAtSetpoint();
    }

    // Individual motor RPM getters for dashboard
    public double getFlywheelMidRPM() {
        return inputs.flywheelMidVelocityRPM;
    }

    public double getFlywheelRightRPM() {
        return inputs.flywheelRightVelocityRPM;
    }

    public double getFlywheel2InRPM() {
        return inputs.flywheel2InVelocityRPM;
    }

    public double getShooterMotorRPM() {
        return inputs.shooterMotorVelocityRPM;
    }

    public double getPooperRPM() {
        return inputs.pooperVelocityRPM;
    }
}
