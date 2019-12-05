package frc.lib5k.components.drive;

import frc.lib5k.utils.Mathutils;

/**
 * A collection of utils for Dealing with joystick inputs (commonly used for
 * controlling robot drivetrain)
 */
public class InputUtils {

    /**
     * A simple way to reduce joystick "Effectiveness" as the outputs near the
     * edges. This can be used if an input is too sensitive but only on the extreme
     * ends of the inputs
     * 
     * @param input          Input value
     * @param scaling_factor How much to reduce the maximum output by (This must be
     *                       < 0.33 * maximum. This method will handle incorrect
     *                       inputs for you)
     * @param max_input      Maximum value expected by the input
     * @param min_input      Minimum value expected by the input
     * @return New, scaled value
     */
    public static double dampenInputs(double input, double scaling_factor, double max_input, double min_input) {
        // Determine the maximum magnitude
        double max_magnitude = Math.max(Math.abs(max_input), Math.abs(min_input));

        // Determine the maximum scaling factor
        // If the scaling factor is greater than .33 of the max magnitude, the output
        // will behave weirdly
        double max_scaling_factor = max_magnitude * (1.0 / 3.0);

        // Clamp the scaling factor (because, yay for safety!)
        scaling_factor = Mathutils.clamp(scaling_factor, 0, max_scaling_factor);

        // We must clamp the input too
        input = Mathutils.clamp(input, max_scaling_factor * -1, max_scaling_factor);

        // Calculate the output modifier, and apply it to the input
        return input * (1 - (scaling_factor * (input * input)));

    }
}
