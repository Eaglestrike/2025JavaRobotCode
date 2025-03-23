package frc.robot.constants;

public final class IntakeConstants {

    public static class Wrist {
        public final static int MOTOR_ID = 10;

        public final static double MAX_VOLTS = 5.0;
        public final static double MANUAL_RATIO = 5.0;
        public final static double GEAR_RATIO = 9.0;

        public final static double kP = 3.0;
        public final static double kI = 0.0;
        public final static double kD = 0.1;
        public final static double kV = 1.3;
        public final static double kA = 0.05;
        public final static double kS = 0.01;
        public final static double kG = -0.42;

        public final static double MOTION_ACCELERATION = 30.0;
        public final static double MOTION_CRUISE_VELOCITY = 20.0;
        public final static double MOTION_JERK = 0;

        public final static double OFFSET = -0.38;

        public final static double INTAKE_POSITION = 2.1;
        public final static double STOW_POSITION = 0.0;
        public final static double EJECT_POSITION = 1;
    }

    public static class Rollers {
        public final static int MOTOR_ID = 36;

        public final static double MAX_VOLTS = 8.0;
        public final static double MANUAL_RATIO = 1.0;
        public final static double INTAKE_CORAL_VOLTS = 8.0;
        public final static double EJECT_VOLTS = -4.0;
    }
}
