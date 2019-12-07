package frc.lib5k;

import frc.lib5k.networking.PortManager;
import frc.lib5k.utils.RobotLogger;

public class Meta {
    public static void startLib5K() {
        // Create the UnifiedLooper
        UnifiedLooper lib5k_looper = new UnifiedLooper();

        // Start the looper
        lib5k_looper.start(0.02);

        // Start the logger
        RobotLogger.getInstance().start(0.02);

        // Allocate FRC ports
        PortManager.getInstance().allocateFRCPorts();
    }
}