package frc.lib5k.components;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.SpeedController;

public abstract class GearBox extends SendableBase implements SpeedController {

    public abstract void set(double speed);

    public abstract EncoderBase getEncoder();

    public abstract void setMotorSafety(boolean motorSafety);

    public abstract int getTicks();

    public void limitCurrent(int peakCurrent, int holdCurrent, int peakDuration_ms) {
        // Default message for non-overridden method
        System.out.println("WARNING: This GearBox does not support current limiting");
    }

}