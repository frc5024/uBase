package frc.lib5k.control;

import frc.lib5k.kinematics.DriveSignal;
import frc.lib5k.kinematics.PIDProfile;
import frc.lib5k.utils.Mathutils;

/**
 * Closed-loop drive correction for differential drivebases
 */
public class DriveCorrector {

    private PIDv2 pidController;
    private boolean isFirstRun = true;
    private double maxOutput = 1.0;

    /**
     * Create a DriveCorrector
     * 
     * @param profile PIDProfile for correction
     */
    public DriveCorrector(PIDProfile profile) {
        pidController = new PIDv2(profile);
    }

    /**
     * Correct a DriveSignal for closed-loop control to stay driving straight at all
     * times, while still giving the drivers control
     * 
     * @param signal  DriveSignal to correct
     * @param angle   Robot gyroscope angle (degrees)
     * @param epsilon How much the robot must try to turn in order to stop
     *                correction, and give drivers control
     * @return Corrected signal
     */
    public DriveSignal correct(DriveSignal signal, double angle, double epsilon) {

        // Wrap the angle
        angle %= 360;

        // Set the corrector setpoint to the current if this is the first calculation
        if (isFirstRun) {
            // Set the setpoint
            pidController.setSetpoint(angle);

            // Disable first run
            isFirstRun = false;
        }

        // Determine velocity difference
        double dv = Math.abs(signal.getL() - signal.getR());

        // Determine the speed
        double speed = (signal.getL() + signal.getR()) / 2;

        // Only modify signal if dv is within epsilon and the signal has speed
        if (Mathutils.epsilonEquals(dv, 0, epsilon) && !Mathutils.epsilonEquals(speed, 0, 0.1)) {

            // Determine the wrapped error, and multiply by -1
            double error = Mathutils.getWrappedError(angle, pidController.getSetpoint()) * -1;

            // Calculate the PID result
            double corrective_factor = pidController.calculate(error);

            // Enforce max corrective factor
            corrective_factor = Mathutils.clamp(corrective_factor, maxOutput * -1, maxOutput);

            // Modify signal by corrective factor
            signal.setL(signal.getL() + corrective_factor);
            signal.setR(signal.getR() - corrective_factor);

            // Re-normalize the signal
            signal.normalize();

        } else {
            // Setting "First Run" to true will cause a re-setting of the setpoint when the
            // robot straightens out
            isFirstRun = false;

        }

        return signal;
    }

    public void configMaxFactor(double max) {
        maxOutput = Mathutils.clamp(max, 0.0, 1.0);
    }
}