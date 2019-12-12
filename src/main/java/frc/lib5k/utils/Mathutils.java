package frc.lib5k.utils;

public class Mathutils {

    public static void main(String[] args) {
        System.out.println(getWrappedError(270, 0.0));
        System.out.println(getWrappedError(0.0, 270.0));
        System.out.println(wrapGyro(10));
        System.out.println(wrapGyro(270));
        System.out.println(wrapGyro(360));
        System.out.println(wrapGyro(420));
        System.out.println(420%360);
    }

    /**
     * This allows the angle to respect 'wrapping', where 360 and 0 are the same
     * value. This will also remap 0 -> 360 to -180 -> 180
     * 
     * @param angle Gyroscope angle
     * @return Wrapped value
     */
    public static double wrapGyro(double angle) {
        // Wrap the angle by 360degs
        angle %= 360.0;

        // Handle offset
        if (Math.abs(angle) > 180.0) {
            angle = (angle > 0) ? angle - 360 : angle + 360;
        }

        return angle;
    }

    /**
     * Gets the error between two angles and allows crossing the 360/0 degree
     * boundary
     * 
     * @param currentAngle Current angle
     * @param desiredAngle Desired/goal angle
     * @return Difference
     */
    public static double getWrappedError(double currentAngle, double desiredAngle) {
        double phi = Math.abs(currentAngle - desiredAngle) % 360; // This is either the distance or 360 - distance
        double distance = phi > 180 ? 360 - phi : phi;

        // Determine the sign (is the difference positive of negative)
        int sign = (currentAngle - desiredAngle >= 0 && currentAngle - desiredAngle <= 180)
                || (currentAngle - desiredAngle <= -180 && currentAngle - desiredAngle >= -360) ? 1 : -1;

        // Return the final difference
        return distance * sign;

    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low   The lower boundary to which to clamp value.
     * @param high  The higher boundary to which to clamp value.
     */
    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * Determine if two values are equal within an acceptable error
     * 
     * @param a   A value
     * @param b   B value
     * @param eps Epsilon
     * @return Is A equal to B within Epsilon?
     */
    public static boolean epsilonEquals(double a, double b, double eps) {
        return (a - eps <= b) && (a + eps >= b);
    }
}