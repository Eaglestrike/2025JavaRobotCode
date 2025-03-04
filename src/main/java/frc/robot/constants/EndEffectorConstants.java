package frc.robot.constants;

public final class EndEffectorConstants {

    public static class Wrist {
        public final static int MOTOR_ID = 0;

        public final static double MAX_VOLTS = 0.0;
        public final static double MANUAL_RATIO = 0.0;
        public final static double GEAR_RATIO = 0.0;

        public final static double kP = 0.0;
        public final static double kI = 0.0;
        public final static double kD = 0.0;
        public final static double kV = 0.0;
        public final static double kA = 0.0;
        public final static double kS = 0.0;
        public final static double kG = 0.0;

        public final static double MOTION_ACCELERATION = 0.0;
        public final static double MOTION_CRUISE_VELOCITY = 0.0;
        public final static double MOTION_JERK = 0.0;

        public final static double OFFSET = 0.0;

        // angles for when preparing to score
        double L1_PRE_ANGLE = 0.0;
        double L2_PRE_ANGLE = 0.0;
        double L3_PRE_ANGLE = 0.0;
        double L4_PRE_ANGLE = 0.0;
        // angles for when scoring
        double L1_SCORE_ANGLE = 0.0;
        double L2_SCORE_ANGLE = 0.0;
        double L3_SCORE_ANGLE = 0.0;
        double L4_SCORE_ANGLE = 0.0;
        // other angles
        double DEALGAE_HIGH_ANGLE = 0.0;
        double DEALGAE_LOW_ANGLE = 0.0;
        double STOW_WRIST_ANGLE = 0.0;
        double INTAKE_CORAL_ANGLE = 0.0;
        double SCORE_LARGE_ANGLE = 0.0;
    }

    public static class Rollers {
        public final static int MOTOR_ID = 0;

        public final static double MAX_VOLTS = 0.0;
        public final static double INTAKE_CORAL_VOLTS = 0.0;
        public final static double INTAKE_ALGAE_VOLTS = 0.0;
        public final static double OUTTAKE_L2_L3_CORAL_VOLTS = 0.0;
        public final static double OUTTAKE_L1_CORAL_VOLTS = 0.0;
        public final static double OUTTAKE_L4_CORAL_VOLTS = 0.0;
        public final static double OUTTAKE_BARGE_VOLTS = 0.0;
        public final static double OUTTAKE_PROCCESOR_VOLTS = 0.0;
        public final static double RETAIN_ALGAE = 0.0;

    }
}
