package frc.lib5k.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonEncoder extends EncoderBase {
    WPI_TalonSRX talon;

    public TalonEncoder(WPI_TalonSRX talon) {
        this.talon = talon;
    }

    @Override
    public int getRawTicks() {
        return talon.getSelectedSensorPosition();
    }
    
    @Override
    public double getRate(double wheelcirc, double tpr) {
        // TODO Auto-generated method stub
        return 0;
    }
}