package frc.lib5k.components.drive;

import frc.lib5k.kinematics.DriveSignal;

public class DriveCalculator {

    public DriveCalculator() {



    }


    public static DriveSignal normalize(DriveSignal signal) {

        // Maximum magnitude between both wheels
        double mag = Math.max(Math.abs(signal.getL()), Math.abs(signal.getR()));

        if (mag > 1.0) {
            signal.setL(signal.getL() / mag);
            signal.setR(signal.getR() / mag);
        }

        return signal;

    }

    public static DriveSignal semiConstCurve(double speed, double rotation) {
        

        
    
        
    }

}