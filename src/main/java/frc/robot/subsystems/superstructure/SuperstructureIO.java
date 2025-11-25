package frc.robot.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLog;

public interface SuperstructureIO {
    @AutoLog
    class SuperstructureIOInputs {
        public Superstructurestate currentState = Superstructurestate.IDLE;
        public Superstructurestate desiredState = Superstructurestate.IDLE;
        public double stateStartTime = 0.0;
        
        //Intake Status
        public boolean intakeHasGamePiece = false;
        public boolean intakeDeployed = false;
        public double intakeArmPosition = 0.0;
        
        //Shooter status stuff, may need to be changed later when actually implemented
        public boolean shooterAtSpeed = false;
        public double shooterVelocity = 0.0;
        
        //Ready status
        public boolean readyToIntake = false;
        public boolean readyToShoot = false;
    }
    
    default void updateInputs(SuperstructureIOInputs inputs) {}
}