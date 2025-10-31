package frc.robot.constants;

public final class EndEffectorConstants {

    public static class Wrist {
        public final static int MOTOR_ID = 14;
        public final static int ENCODER_ID = 10;

        public final static double MAX_VOLTS = 3;
        public final static double MANUAL_RATIO = 3;
        public final static double SENSOR_TO_MECHANISM_RATIO = 0.26;
        public final static double ROTOR_TO_SENSOR_RATIO = 1;

        public final static double kP = 20;
        public final static double kI = 0.0;
        public final static double kD = 1;
        public final static double kV = 0.0;
        public final static double kA = 0.0;
        public final static double kS = 0.0;
        public final static double kG = 0.0;

        public final static double MOTION_ACCELERATION = 15;
        public final static double MOTION_CRUISE_VELOCITY = 15;
        public final static double MOTION_JERK = 0.0;

        public final static double OFFSET = 0;
    }

    public static class Rollers {
        public final static int MOTOR_ID = 34;

        public final static double MAX_VOLTS = 8.0;
        public final static double MANUAL_RATIO = 4.0;
        public final static double INTAKE_CORAL_VOLTS = 5;
        public final static double INTAKE_ALGAE_VOLTS = 8.0;
        public final static double OUTTAKE_L2_L3_CORAL_VOLTS = 0.0;
        public final static double OUTTAKE_L1_CORAL_VOLTS = -4;
        public final static double OUTTAKE_L4_CORAL_VOLTS = -2.0;
        public final static double OUTTAKE_BARGE_VOLTS = -12.0;
        public final static double OUTTAKE_PROCCESOR_VOLTS = -8.0;
        public final static double RETAIN_ALGAE = 2.0;
        public final static double RETAIN_CORAL = 0.3;
        public final static double EJECT_VOLTS = -5;
        public final static double DEALGAE_VOLTS = -5;

        public final static double STALL_CURRENT = 30;

    }
}
