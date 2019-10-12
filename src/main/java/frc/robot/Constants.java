package frc.robot;

import frc.lib5k.kinematics.PIDProfile;

public class Constants {

    /* DriveTrain */
    public static class DriveTrain {
        public static final int leftFrontMotor = 1;
        public static final int leftRearMotor = 2;
        public static final int rightFrontMotor = 3;
        public static final int rightRearMotor = 4;

        public static final int peakCurrent = 35;
        public static final int holdCurrent = 33;
        public static final int currentTimeout = 30;

        public static final int maxVelocity = 0; // Ticks per 100ms

        public static PIDProfile driftCorrectionGains = new PIDProfile(1.0, 0.0, 0.0);
        public static PIDProfile forwardPIDGains = new PIDProfile(1.0, 0.0, 0.0);
        public static PIDProfile turnPIDGains = new PIDProfile(1.0, 0.0, 0.0);

        public static final int ticksPerRotation = 360;
    }
    
    public class Robot {
        public static final double wheelDiameter = 6.0 * 2.54; // Wheel is in inches.. Convert to cm
        public static final double wheelCirc = Math.PI * wheelDiameter;
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