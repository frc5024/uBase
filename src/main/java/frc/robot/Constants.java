package frc.robot;

public class Constants {

    /* DriveTrain */
    public class DriveTrain {
        public static final int leftFrontMotor = 1;
        public static final int leftRearMotor = 2;
        public static final int rightFrontMotor = 3;
        public static final int rightRearMotor = 4;

        public static final int peakCurrent = 35;
        public static final int holdCurrent = 33;
        public static final int currentTimeout = 30;

        public static final int maxVelocity = 0; // Ticks per 100ms

        public class DriftCorrection {
            public static final double kp = 1.0;
            public static final double ki = 0.0;
            public static final double kd = 0.0;
        }
    }

    public class Deadbands {
        public static final double rotation_deadband = 0.1;
        public static final double roataion_percision = 0.2;
        public static final double speed_percision = 0.1;
        public static final double slider_deadband = 0.1;
    }

    /* DriveControl */
    public static final double accelerationStep = 0.2;
}