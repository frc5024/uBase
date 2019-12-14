package frc.robot.subsystems.grip;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib5k.components.motors.TalonSRXCollection;
import frc.lib5k.components.sensors.EncoderBase;
import frc.lib5k.utils.OnNewBoolean;
import frc.lib5k.utils.RobotLogger;
import frc.lib5k.utils.RobotLogger.Level;
import frc.robot.Constants;

public class GripSystem extends Subsystem {
    RobotLogger logger = RobotLogger.getInstance();
    private static GripSystem m_instance = null;

    // Subsystem state
    public enum GripState {
        kAcceptBall("Accept Ball"), KGrip("Grip"), KShoot("Shoot"), KStow("Stow");

        private String name;

        GripState(String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    /* Constants */
    public final int MAX_RPM = 1630;

    DoubleSolenoid m_switcher;
    TalonSRXCollection m_shooter;
    EncoderBase m_leftGripSensor, m_rightGripSensor;
    DigitalInput m_ballSensor;

    GripState m_wantedState = GripState.KStow;

    /* System wants */
    boolean m_hasBall = false;
    OnNewBoolean m_shouldGrip = new OnNewBoolean();

    int m_targetRPM = 0;

    private GripSystem() {
        logger.log("GripSystem", "Initializing", Level.kRobot);

        m_switcher = new DoubleSolenoid(Constants.pcm_id, 1, 2);
        m_shooter = new TalonSRXCollection(new WPI_TalonSRX(5), new WPI_TalonSRX(6));
        m_leftGripSensor = m_shooter.getEncoder(0);
        m_rightGripSensor = m_shooter.getEncoder(1);
        m_ballSensor = new DigitalInput(0);

        // Config shooter
        m_shooter.getMaster().setNeutralMode(NeutralMode.Coast);
    }

    public static GripSystem getInstance() {
        if (m_instance == null) {
            m_instance = new GripSystem();
        }

        return m_instance;
    }

    @Override
    public void periodic() {

        // Determing shooter RPM
        // int rpm = (m_leftGripSensor.getSpeed() + m_rightGripSensor.getSpeed()) / 2;
        double rmp = (((m_shooter.getMaster().getSelectedSensorVelocity() / 100)
                + (m_shooter.getSlave(0).getSelectedSensorVelocity() / 100))) * 60000;

        // switch (m_wantedState) {
        // case KStow:
        // // Set the solenoids to the flywheel side
        // m_switcher.set(Value.kReverse);

        // // Stop the motors
        // m_shooter.set(0.0);
        // break;

        // case kAcceptBall:
        // // Set the solenoids to the flywheel side
        // m_switcher.set(Value.kReverse);

        // // Intake
        // m_shooter.set(-0.35);
        // break;

        // case KGrip:

        // }

        /* Read sensors */

        // Ball sensor
        m_hasBall = m_ballSensor.get();

        /* State handlers */

        // Handle grip state requests
        if (m_shouldGrip.hasNewData()) {

            // Set the grip based on data
            Value gripValue = (m_shouldGrip.read()) ? Value.kForward : Value.kReverse;

            // Send data to solenoid
            m_switcher.set(gripValue);

            // Mark the data as "read"
            m_shouldGrip.markRead();
        }

    }

    public void setWantedState(GripState state) {
        logger.log("GripSystem", String.format("Wanted state set to: %s", m_wantedState.toString()));
        m_wantedState = state;
    }

    public void setWantedRPM(int rpm) {
        logger.log("GripSystem", String.format("Wanted RPM set to: %d", rpm));
        m_targetRPM = rpm;
    }

    public void setGrip(boolean gripping) {
        logger.log("GripSystem", String.format("Gripper set: %s", (gripping) ? "Gripping" : "Not gripping"));
        m_shouldGrip.set(gripping);
    }

    public boolean hasBall() {
        return m_hasBall;
    }

    @Override
    protected void initDefaultCommand() {
        // TODO Auto-generated method stub

    }

}