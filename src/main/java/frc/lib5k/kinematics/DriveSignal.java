package frc.lib5k.kinematics;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Differential drive motor info
 */
public class DriveSignal {

    public enum DriveType {
        STANDARD, VELOCITY
    }

    private double l, r;
    private NeutralMode mode;
    private DriveType type;

    /**
     * Create a DriveSignal from Left and Right wheel outputs
     * 
     * @param l Left output
     * @param r Right output
     */
    public DriveSignal(double l, double r) {
        this(l, r, DriveType.STANDARD);
    }

    /**
     * Create a DriveSignal from Left and Right wheel outputs and a drive type
     * 
     * @param l    Left output
     * @param r    Right output
     * @param type Drive type
     */
    public DriveSignal(double l, double r, DriveType type) {
        this(l, r, NeutralMode.Brake, type);
    }

    /**
     * Create a DriveSignal from Left and Right wheel outputs, a NeutralMode, and a
     * drive type
     * 
     * @param l    Left output
     * @param r    Right output
     * @param mode Output NeutralMode
     * @param type Drive type
     */
    public DriveSignal(double l, double r, NeutralMode mode, DriveType type) {
        this.l = l;
        this.r = r;
        this.mode = mode;
        this.type = type;
    }

    /**
     * Create a DriveSignal from ArcadeDrive inputs. This will do a direct
     * calculation with no smoothing.
     * 
     * @param speed    Arcade speed
     * @param rotation Arcade turning factor
     * @return Generated DriveSignal
     */
    public static DriveSignal fromArcadeInputs(double speed, double rotation) {
        DriveSignal signal = new DriveSignal((rotation + speed), (speed - rotation));
        signal.normalize();
        return signal;
    }

    /**
     * Get right speed
     * 
     * @return Right speed
     */
    public double getR() {
        return r;
    }

    /**
     * Set right speed
     * 
     * @param r Right speed
     */
    public void setR(double r) {
        this.r = r;
    }

    /**
     * Get left speed
     * 
     * @return Left speed
     */
    public double getL() {
        return l;
    }

    /**
     * Set the left speed
     * 
     * @param l Left speed
     */
    public void setL(double l) {
        this.l = l;
    }

    public NeutralMode getMode() {
        return mode;
    }

    public void setMode(NeutralMode mode) {
        this.mode = mode;
    }

    public DriveType getType() {
        return type;
    }

    public void setType(DriveType type) {
        this.type = type;
    }

    /**
     * Normalize the signal. This will pull back a motor if the other tries to
     * output > 1.0
     */
    public void normalize() {

        // Find the maximum magnitude between both wheels
        double magnitude = Math.max(Math.abs(this.getL()), Math.abs(this.getR()));

        // Scale back motors if the max magnitude is greater than the max output (1.0)
        if (magnitude > 1.) {
            this.setL(this.getL() / magnitude);
            this.setR(this.getR() / magnitude);
        }

    }

}