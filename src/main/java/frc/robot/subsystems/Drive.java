package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib5k.components.GearBox;
import frc.lib5k.control.PID;
import frc.lib5k.control.SlewLimiter;
import frc.lib5k.loops.loopables.LoopableSubsystem;
import frc.lib5k.utils.RobotLogger;
import frc.lib5k.utils.RobotLogger.Level;
import frc.robot.Constants;

public class Drive extends LoopableSubsystem {

    public enum ControlType {
        DEFAULT, ASSIST, AUTONOMOUS, PATH
    }

    RobotLogger logger = RobotLogger.getInstance();

    GearBox m_leftGearbox;
    GearBox m_rightGearbox;

    DifferentialDrive m_differentialDrive;
    PID m_driveCorrector;
    SlewLimiter m_speedSlew;

    boolean m_driftCorrectionActive = false;
    boolean m_isMoving, m_isTurning = false;
    boolean m_isNewConfigData = false;

    NeutralMode m_desiredBrakeMode = NeutralMode.Coast;

    ControlType m_currentControlType = ControlType.DEFAULT;

    public Drive() {
        logger.log("Building drive", Level.kRobot);

        // Build drive gearboxes
        m_leftGearbox = new GearBox(new WPI_TalonSRX(Constants.DriveTrain.leftFrontMotor),
                new WPI_TalonSRX(Constants.DriveTrain.leftRearMotor), true);
        m_rightGearbox = new GearBox(new WPI_TalonSRX(Constants.DriveTrain.rightFrontMotor),
                new WPI_TalonSRX(Constants.DriveTrain.rightRearMotor), true);

        // Config current limits
        m_leftGearbox.limitCurrent(Constants.DriveTrain.peakCurrent, Constants.DriveTrain.holdCurrent,
                Constants.DriveTrain.currentTimeout);
        m_rightGearbox.limitCurrent(Constants.DriveTrain.peakCurrent, Constants.DriveTrain.holdCurrent,
                Constants.DriveTrain.currentTimeout);

        // Build drivebase
        m_differentialDrive = new DifferentialDrive(m_leftGearbox.getMaster(), m_rightGearbox.getMaster());

        // Configure drift PID controller
        m_driveCorrector = new PID(Constants.DriveTrain.DriftCorrection.kp, Constants.DriveTrain.DriftCorrection.ki,
                Constants.DriveTrain.DriftCorrection.kd);
        
        // Configure Slew limiter
        m_speedSlew = new SlewLimiter(Constants.accelerationStep);

    }

    @Override
    public synchronized void periodicInput() {
    }

    @Override
    public synchronized void periodicOutput() {

        // Handle talon config data
        if (m_isNewConfigData) {

            // Set brake mode for all talons
            m_leftGearbox.front.setNeutralMode(m_desiredBrakeMode);
            m_leftGearbox.rear.setNeutralMode(m_desiredBrakeMode);
            m_rightGearbox.front.setNeutralMode(m_desiredBrakeMode);
            m_rightGearbox.rear.setNeutralMode(m_desiredBrakeMode);

            // Data had been sent, disable lock
            m_isNewConfigData = false;
        }
    }

    public void handleDefaultDrive(double speed, double rotation, boolean quickTurn, boolean invertControl) {
        // Ensure this method is allowed
        if (m_currentControlType == ControlType.DEFAULT) {
            smoothDrive(speed, rotation, quickTurn, invertControl);
        }
    }

    /**
     * Drive the robot with some help from sensors
     * 
     * @param speed     Desired robot speed
     * @param rotation  Desired robot turn rate
     * @param quickTurn Should quickTurn be enabled?
     */
    public void smoothDrive(double speed, double rotation, boolean quickTurn, boolean invertControl) {

        // Check if we should be correcting robot drift
        if (rotation == 0.0) {
            double current_angle = Gyroscope.getInstance().getGyro().getAngle();

            // Set drift correction and reset setpoint if this state is new
            if (!m_driftCorrectionActive) {

                m_driveCorrector.setSetpoint(current_angle);

                // Set state
                m_driftCorrectionActive = true;
            }

            // Determine rotation correction
            rotation = m_driveCorrector.feed(current_angle);
        } else {
            // Disable drift correction
            m_driftCorrectionActive = false;
        }

        // Handle inverse control
        speed = (invertControl) ? speed * -1 : speed;

        // Slew robot speed
        speed = m_speedSlew.feed(speed);

        // Set moving and turning trackers
        m_isMoving = (speed != 0.0);
        m_isTurning = (rotation != 0.0);

        // Feed drive command
        m_differentialDrive.curvatureDrive(speed, rotation, quickTurn);

    }

    /**
     * Enables or disables brake mode on all drivebase talons.
     * 
     * @param on Should the brakes be enabled?
     */
    public void setBrakes(boolean on) {
        m_desiredBrakeMode = on ? NeutralMode.Brake : NeutralMode.Coast;
        String mode_string = on ? "Brake" : "Coast";

        logger.log("[DriveTrain] NeutralMode has been set to: " + mode_string);

        // Force an update
        m_isNewConfigData = true;

    }

    public void setMode(ControlType type) {
        m_currentControlType = type;
    }

    /**
     * Directly drive the gearboxes. This should only be used wile motion profiling
     * 
     * @param l Left speed
     * @param r Right speed
     */
    public void rawDrive(double l, double r) {
        m_isMoving = (l + r != 0.0);
        m_isTurning = (l != r);
        m_leftGearbox.set(l);
        m_rightGearbox.set(r);
    }

    /**
     * Get the number of ticks recorded by the left GearBox's encoder.
     * 
     * @return Number of ticks
     */
    public int getLeftGearboxTicks() {
        return m_leftGearbox.getTicks();
    }

    /**
     * Get the number of ticks recorded by the right GearBox's encoder.
     * 
     * @return Number of ticks
     */
    public int getRightGearboxTicks() {
        return m_rightGearbox.getTicks();
    }

    @Override
    public void outputTelemetry() {

        SmartDashboard.putNumber("[DriveTrain] Left gearbox sensor", getLeftGearboxTicks());
        SmartDashboard.putNumber("[DriveTrain] Right gearbox sensor", getRightGearboxTicks());

    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub

    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub

    }

}