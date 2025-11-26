package frc.robot.subsystems.superstructure;

public enum Superstructurestate {
    IDLE,
    INTAKING,
    OUTTAKING,
    STOWED,
    DEPLOYED,
    SHOOTING,
    TRANSITIONING,
    PREPARING_TO_SHOOT;

    public boolean isIntakeDeployed() {
        return this == DEPLOYED || this == STOWED;
    }

    public boolean isAquiringPiece() {
        return this == INTAKING;
    }
}
