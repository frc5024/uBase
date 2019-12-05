package frc.lib5k.utils;

public class InputUtils{
    public enum ScalingMode{
        LINEAR, SQUARED, CUBIC, QUARTIC;
    }

    /**
     * Scales Input Value by Desired Factor
     * @param val Value
     * @param mode Factor to scale by (val^mode essentially)
     * @return Scaled value
     */
    public static double scale(double val, ScalingMode mode){
        switch(mode){
            case LINEAR:
                return val;
            case SQUARED:
                return Math.copySign(val*val, val);
            case CUBIC:
                return val*val*val;
            case QUARTIC: 
                return Math.copySign(val*val*val*val, val);
            default:
                return 0.0;
        }
    }
}