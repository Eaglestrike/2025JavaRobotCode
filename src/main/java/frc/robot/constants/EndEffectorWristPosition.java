package frc.robot.constants;

/**
 * In terms of the number of rotations using the encoder
 */
public enum EndEffectorWristPosition {
    // coral
    L2_PRE_ANGLE(1.85),
    L3_PRE_ANGLE(3),
    L4_PRE_ANGLE(3),
    L1_SCORE_ANGLE(1.3),
    L2_SCORE_ANGLE(0.0),
    L3_SCORE_ANGLE(0.0),
    L4_SCORE_ANGLE(1),
    L4_SCORE_NO_DEALGAE_ANGLE(1.6),
    INTAKE_CORAL_ANGLE(0.0), // should be the same as STOW_ANGLE
    // algae
    SCORE_BARGE_ANGLE(3),
    SCORE_BARGE_PRE_ANGLE(3),
    SCORE_PROCESSOR_ANGLE(0.8),
    INTAKE_ALGAE_ANGLE_L3(1.7),
    INTAKE_ALGAE_ANGLE_L2(1.6),
    // stow
    STOW_ANGLE(0.0);

    private final double angle;

    EndEffectorWristPosition(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }

    public double getBackAngle() {
        return -angle;
    }
}
