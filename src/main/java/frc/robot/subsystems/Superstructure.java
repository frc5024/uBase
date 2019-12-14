package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib5k.interfaces.Loggable;
import frc.lib5k.utils.RobotLogger;
import frc.robot.subsystems.grip.GripSystem;

/**
 * An interface for all subsystems. The superstructure handles collision
 * detection, and coordination
 */
public class Superstructure extends Subsystem implements Loggable {
    private static Superstructure m_instance = null;
    private RobotLogger logger = RobotLogger.getInstance();

    GripSystem m_grip = GripSystem.getInstance();
    Drive m_drive = Drive.getInstance();

    private Superstructure() {

    }

    public static Superstructure getInstance() {
        if (m_instance == null) {
            m_instance = new Superstructure();

        }

        return m_instance;
    }

    @Override
    public void periodic() {

    }

    

    @Override
    public void logStatus() {
        // TODO Auto-generated method stub

    }

    @Override
    public void updateTelemetry() {
        // TODO Auto-generated method stub

    }

    @Override
    protected void initDefaultCommand() {
    }

}