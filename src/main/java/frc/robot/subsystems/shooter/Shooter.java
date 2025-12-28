package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double targetFlywheelRPM = 0.0;
    private double targetFlywheel2InRPM = 0.0;
    private double targetHoodPosition = 0.0;
    private boolean shooterEnabled = false;

    // Mechanism2d for visualization
    private final Mechanism2d mechanism;
    private final MechanismLigament2d hoodLigament;
    private final MechanismLigament2d flywheelLeftLigament;
    private final MechanismLigament2d flywheelRightLigament;
    private final MechanismLigament2d flywheel2InLigament;
    private final MechanismLigament2d indexerLigament;
    private final MechanismLigament2d pooperLigament;

    // Flywheel spinner animation
    private double flywheelSpinAngle = 0.0;
    private double flywheel2InSpinAngle = 0.0;
    private double indexerSpinAngle = 0.0;
    private double pooperSpinAngle = 0.0;

    public Shooter(ShooterIO io) {
        this.io = io;

        // Create Mechanism2d (width, height in meters)
        mechanism = new Mechanism2d(4, 3);

        // Create shooter base/frame
        MechanismRoot2d shooterBase = mechanism.getRoot("ShooterBase", 2.0, 0.5);

        // Hood pivot - rotates to adjust shot angle
        hoodLigament = shooterBase.append(new MechanismLigament2d(
                "Hood",
                0.4, // Hood length
                45, // Starting angle (degrees)
                8,
                new Color8Bit(Color.kBlue)));

        // Left flywheel (FlywheelMid) - at end of hood
        MechanismRoot2d flywheelLeftRoot = mechanism.getRoot("FlywheelLeftRoot", 1.5, 1.2);
        flywheelLeftLigament =
                flywheelLeftRoot.append(new MechanismLigament2d("FlywheelLeft", 0.15, 0, 6, new Color8Bit(Color.kRed)));

        // Right flywheel (FlywheelRight)
        MechanismRoot2d flywheelRightRoot = mechanism.getRoot("FlywheelRightRoot", 2.5, 1.2);
        flywheelRightLigament = flywheelRightRoot.append(
                new MechanismLigament2d("FlywheelRight", 0.15, 0, 6, new Color8Bit(Color.kRed)));

        // 2" Flywheel - feeds into hood flywheels
        MechanismRoot2d flywheel2InRoot = mechanism.getRoot("Flywheel2InRoot", 2.0, 0.8);
        flywheel2InLigament =
                flywheel2InRoot.append(new MechanismLigament2d("Flywheel2In", 0.1, 0, 5, new Color8Bit(Color.kOrange)));

        // Indexer (starwheels)
        MechanismRoot2d indexerRoot = mechanism.getRoot("IndexerRoot", 1.2, 0.5);
        indexerLigament =
                indexerRoot.append(new MechanismLigament2d("Indexer", 0.12, 0, 5, new Color8Bit(Color.kYellow)));

        // Pooper
        MechanismRoot2d pooperRoot = mechanism.getRoot("PooperRoot", 2.8, 0.5);
        pooperLigament = pooperRoot.append(new MechanismLigament2d("Pooper", 0.12, 0, 5, new Color8Bit(Color.kPurple)));

        SmartDashboard.putData("Shooter/Mechanism", mechanism);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Control shooter based on enable state
        if (shooterEnabled && targetFlywheelRPM > 0) {
            io.setFlywheelVelocity(targetFlywheelRPM);
            io.setFlywheel2InVelocity(targetFlywheel2InRPM);
            io.setHoodPosition(targetHoodPosition);
        } else if (!shooterEnabled) {
            io.stopFlywheels();
            io.stopHood();
        }

        // Log shooter state
        Logger.recordOutput("Shooter/TargetFlywheelRPM", targetFlywheelRPM);
        Logger.recordOutput("Shooter/TargetFlywheel2InRPM", targetFlywheel2InRPM);
        Logger.recordOutput("Shooter/TargetHoodAngle", targetHoodPosition);
        Logger.recordOutput("Shooter/Enabled", shooterEnabled);
        Logger.recordOutput("Shooter/FlywheelAtSetpoint", flywheelAtSetpoint());
        Logger.recordOutput("Shooter/HoodAtSetpoint", isHoodAtSetpoint());

        // Update mechanism visualization
        updateMechanism();
    }

    private void updateMechanism() {
        // Update hood angle
        hoodLigament.setAngle(inputs.hoodPositionDegrees + 45); // Offset for visual representation

        // Animate flywheels based on velocity (scale RPM to rotation speed)
        double flywheelRotationRate = inputs.velocityRPM / 60.0 * 360.0 * 0.02; // degrees per 20ms
        flywheelSpinAngle += flywheelRotationRate;
        flywheelSpinAngle %= 360;
        flywheelLeftLigament.setAngle(flywheelSpinAngle);
        flywheelRightLigament.setAngle(-flywheelSpinAngle); // Opposite direction

        // Animate 2" flywheel
        double flywheel2InRotationRate = inputs.flywheel2InVelocityRPM / 60.0 * 360.0 * 0.02;
        flywheel2InSpinAngle += flywheel2InRotationRate;
        flywheel2InSpinAngle %= 360;
        flywheel2InLigament.setAngle(flywheel2InSpinAngle);

        // Animate indexer (starwheels)
        double indexerRotationRate = inputs.shooterMotorVelocityRPM / 60.0 * 360.0 * 0.02;
        indexerSpinAngle += indexerRotationRate;
        indexerSpinAngle %= 360;
        indexerLigament.setAngle(indexerSpinAngle);

        // Animate pooper
        double pooperRotationRate = inputs.pooperVelocityRPM / 60.0 * 360.0 * 0.02;
        pooperSpinAngle += pooperRotationRate;
        pooperSpinAngle %= 360;
        pooperLigament.setAngle(pooperSpinAngle);

        // Update colors based on state
        if (shooterEnabled) {
            // Green when at setpoint, yellow when spinning up
            if (flywheelAtSetpoint()) {
                flywheelLeftLigament.setColor(new Color8Bit(Color.kGreen));
                flywheelRightLigament.setColor(new Color8Bit(Color.kGreen));
            } else {
                flywheelLeftLigament.setColor(new Color8Bit(Color.kYellow));
                flywheelRightLigament.setColor(new Color8Bit(Color.kYellow));
            }

            // Hood color - green when at setpoint
            if (isHoodAtSetpoint()) {
                hoodLigament.setColor(new Color8Bit(Color.kGreen));
            } else {
                hoodLigament.setColor(new Color8Bit(Color.kBlue));
            }
        } else {
            // Red when disabled
            flywheelLeftLigament.setColor(new Color8Bit(Color.kRed));
            flywheelRightLigament.setColor(new Color8Bit(Color.kRed));
            hoodLigament.setColor(new Color8Bit(Color.kBlue));
        }

        // 2" flywheel color
        if (Math.abs(inputs.flywheel2InVelocityRPM) > 100) {
            flywheel2InLigament.setColor(new Color8Bit(Color.kOrange));
        } else {
            flywheel2InLigament.setColor(new Color8Bit(Color.kGray));
        }

        // Indexer color - yellow when running
        if (Math.abs(inputs.shooterMotorVelocityRPM) > 100) {
            indexerLigament.setColor(new Color8Bit(Color.kYellow));
        } else {
            indexerLigament.setColor(new Color8Bit(Color.kGray));
        }

        // Pooper color - purple when running
        if (Math.abs(inputs.pooperVelocityRPM) > 100) {
            pooperLigament.setColor(new Color8Bit(Color.kPurple));
        } else {
            pooperLigament.setColor(new Color8Bit(Color.kGray));
        }
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

    public double getTargetHoodPosition() {
        return targetHoodPosition;
    }

    public void enable() {
        shooterEnabled = true;
    }

    public void disable() {
        shooterEnabled = false;
        targetFlywheelRPM = 0.0;
        targetFlywheel2InRPM = 0.0;
        targetHoodPosition = 1;
    }

    public boolean flywheelAtSetpoint() {
        if (!shooterEnabled || targetFlywheelRPM == 0) {
            return false;
        }
        double error = Math.abs(inputs.velocityRPM - targetFlywheelRPM);
        return error < 1; // Within 100 RPM
    }

    public double getFlywheelRPM() {
        return inputs.velocityRPM;
    }

    public boolean isEnabled() {
        return shooterEnabled;
    }

    // Hood control methods
    public void setHoodPosition(double position) {
        this.targetHoodPosition = position;
    }

    public double getHoodAngle() {
        return inputs.hoodPositionDegrees;
    }

    public boolean isHoodAtSetpoint() {
        return inputs.hoodAtSetpoint;
    }

    // Indexer control (starwheels)
    public void runIndexer(double velocityRPS) {
        io.setIndexerVelocity(velocityRPS);
    }

    public void stopIndexer() {
        io.stopIndexer();
    }

    // Pooper control
    public void ejectBall() {
        io.setPooperVelocity(50.0); // ~3000 RPM
    }

    public void feedBall() {
        io.setPooperVelocity(50.0); // ~3000 RPM
    }

    public void stopPooper() {
        io.stopPooper();
    }

    // Configure shooter for different shots
    public void configureShot(double flywheelRPM, double flywheel2InRPM, double hoodPosition) {
        setTargetFlywheelRPM(flywheelRPM);
        setTargetFlywheel2InRPM(flywheel2InRPM);
        setHoodPosition(hoodPosition);
    }

    // Preset shot configurations
    public void configureTestShot() {
        configureShot(60, 150, 5.5); // Hood flywheels, 2in flywheel, hood angle
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

    // Get the Mechanism2d for dashboard display
    public Mechanism2d getMechanism() {
        return mechanism;
    }
}
