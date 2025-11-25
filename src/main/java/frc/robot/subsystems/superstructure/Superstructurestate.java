package frc.robot.subsystems.superstructure;

public enum Superstructurestate {
    IDLE,
    INTAKING,
    OUTTAKING,
    STOWED,
    DEPLOYED,
    SHOOTING,
    HOLDING_PIECE,
    TRANSITIONING,
    PREPARING_TO_SHOOT;

    public boolean isIntakeDeployed() {
        return this == DEPLOYED || this == STOWED;
    }

    public boolean hasGamePiece() {
        return this == HOLDING_PIECE || this == PREPARING_TO_SHOOT || this == SHOOTING;
    }

    public boolean isAquiringPiece() {
        return this == INTAKING;
    }
}
