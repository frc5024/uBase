package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.lib5k.utils.RobotLogger;
import frc.lib5k.utils.RobotLogger.Level;

public class Gyroscope {
    static Gyroscope m_instance = null;
    RobotLogger logger = RobotLogger.getInstance();

    AHRS m_gyro;

    double autonOffset = 0.0;

    public Gyroscope() {
        logger.log("[Gyroscope] Attaching to MXP gyro", Level.kRobot);
        m_gyro = new AHRS(Port.kMXP);

        Shuffleboard.getTab("DriverStation").add(m_gyro);

    }

    public static Gyroscope getInstance() {
        if (m_instance == null) {
            m_instance = new Gyroscope();
        }

        return m_instance;
    }

    public AHRS getGyro() {
        return m_gyro;
    }

    /**
     * Set the offset / error for the gyro from robot's position at the start of
     * autonomous. Never call this unless it is form Robot.autonomousInit()
     */
    public void setAutonOffset() {
        autonOffset = m_gyro.getAngle();
    }

    public void overrideAutonOffset(double angle) {
        autonOffset = angle;
    }

    public double getAutonOffset() {
        return autonOffset;
    }

}