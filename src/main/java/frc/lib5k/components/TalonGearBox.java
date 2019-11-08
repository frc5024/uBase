package frc.lib5k.components;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * A GearBox is a wrapper for any pair of WPI_TalonSRX motor controllers where
 * the first controller has an encoder attached.
 */
public class TalonGearBox extends GearBox {
    private ArrayList<WPI_TalonSRX> motors = new ArrayList<WPI_TalonSRX>();
    private int encoderID;

    public TalonGearBox(WPI_TalonSRX... motors) {
        this(0, motors);
    }

    /**
     * Constructor for a GearBox made of WPI_TalonSRX objects
     * @param encoderPort
     * @param motors
     */
    public TalonGearBox(int encoderPort, WPI_TalonSRX... motors) {
        // Add each motor to the list
        for (WPI_TalonSRX m : motors) {
            this.motors.add(m);
        }

        this.encoderID = encoderPort;

        // Find the first Talon, and use it as the master
        int firstDevID = this.motors.get(0).getDeviceID();

        // Configure each Talon
        this.motors.forEach((m) -> {

            // Factory-reset each talon
            m.configFactoryDefault();

            // Slave each talon to the first
            if (m.getDeviceID() != firstDevID) {
                m.follow(this.motors.get(0));
            }

            // Enable voltage compensation
            m.enableVoltageCompensation(true);

        });

        // Configure the encoder controller, and handle the option for no encoder
        if (0 <= encoderID && encoderID < this.motors.size()) {
            this.motors.get(encoderPort).configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        } else {
            encoderID = -1;
        }

    }

    /**
     * Wrapper method around the WPI_TalonSRX current limiting functionality
     * 
     * @param peakCurrent  The current threshold that must be passed before the
     *                     limiter kicks in
     * @param holdCurrent  The current to hold the motors at once the threshold has
     *                     been passed
     * @param peakDuration The duration of the current limit
     */
    @Override
    public void limitCurrent(int peakCurrent, int holdCurrent, int peakDuration_ms) {
        int timeout = 0;

        // Configure current limiting for each motor
        this.motors.forEach((m) -> {
            m.configPeakCurrentLimit(peakCurrent, timeout);
            m.configPeakCurrentDuration(peakDuration_ms, timeout);
            m.configContinuousCurrentLimit(holdCurrent, timeout);
            m.enableCurrentLimit(true);
        });
    }

    /**
     * Wrapper around the encoder for the front or master talon
     * 
     * @return Number of ticks reported by the front or master talon
     */
    @Override
    public int getTicks() {
        return (encoderID != -1) ? motors.get(encoderID).getSelectedSensorPosition() : 0;
    }

    /**
     * Get the master controller
     * 
     * @return The master motor controller
     */
    public WPI_TalonSRX getMaster() {
        return motors.get(0);
    }

    /**
     * Set the GearBox speed
     * 
     * @param speed Percent output
     */
    @Override
    public void set(double speed) {
        motors.get(0).set(speed);
    }

    @Override
    public EncoderBase getEncoder() {
        return new GearBoxEncoder(this);
    }

    @Override
    public void setMotorSafety(boolean motorSafety) {
        motors.forEach((m) -> {

            // Set motor safety
            m.setSafetyEnabled(motorSafety);
        });

    }
}
