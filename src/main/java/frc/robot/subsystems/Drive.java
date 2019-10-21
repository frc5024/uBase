package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib5k.components.EncoderBase;
import frc.lib5k.components.GearBox;
import frc.lib5k.components.GearBoxEncoder;
import frc.lib5k.control.PID;
import frc.lib5k.control.PIDv2;
import frc.lib5k.control.SlewLimiter;
import frc.lib5k.kinematics.DriveConstraints;
import frc.lib5k.kinematics.DriveSignal;
import frc.lib5k.kinematics.FieldPosition;
import frc.lib5k.kinematics.DriveSignal.DriveType;
import frc.lib5k.utils.RobotLogger;
import frc.lib5k.utils.RobotLogger.Level;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveControl;
import frc.team1114.SimPoint;

public class Drive extends Subsystem {

    // public enum ControlType {
    // DEFAULT, ASSIST, AUTONOMOUS, PATH
    // }

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
    private int m_leftEncoderOffset, m_rightEncoderOffset = 0;

    // ControlType m_currentControlType = ControlType.DEFAULT;

    // PID controllers for pathing
    PIDv2 m_forwardController;
    PIDv2 m_turnController;

    // Encoders
    EncoderBase m_leftEncoder;
    EncoderBase m_rightEncoder;

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
        m_differentialDrive.setSafetyEnabled(false);
        m_leftGearbox.getMaster().setSafetyEnabled(false);
        m_rightGearbox.getMaster().setSafetyEnabled(false);

        // Configure drift PID controller
        m_driveCorrector = new PID(Constants.DriveTrain.turnPIDGains);

        // Configure Slew limiter
        m_speedSlew = new SlewLimiter(Constants.accelerationStep);

        // Configure PID controllers for pathing
        m_forwardController = new PIDv2(Constants.DriveTrain.forwardPIDGains);
        m_turnController = new PIDv2(Constants.DriveTrain.turnPIDGains);

        // Configure encoders
        m_leftEncoder = new GearBoxEncoder(m_leftGearbox);
        m_rightEncoder = new GearBoxEncoder(m_rightGearbox);

        // Send PID controllers
        Shuffleboard.getTab("DriverStation").add("ForwardPID", m_forwardController);
        Shuffleboard.getTab("DriverStation").add("TurnPID", m_turnController);
    }

    @Override
    public void periodic() {
        // Update encoders
        m_leftEncoder.update();
        m_rightEncoder.update();

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

        // Output telemetry data
        outputTelemetry();

    }

    /**
     * Drive the robot to a relative point in space. Note: If driving backwards, the
     * minimum velocity must be less than 0
     * 
     * @param end         Point to drive to
     * @param constraints Robot kinematic constraints
     * @param turnRate    Rate to release turning constraints per 20ms (low numbers
     *                    will take longer to turn, and create a long arc)
     * @param epsilon     Error around end point
     * 
     * @return Has the action finished yet
     */
    public boolean driveTo(FieldPosition end, DriveConstraints constraints, double turnRate, double epsilon) {

        // Configure PID constraints
        m_forwardController.setOutputConstraints(constraints.getMinVel(), constraints.getMaxVel()); 
        // m_forwardController.setOutputConstraints(-constraints.getMaxVel(),
        // constraints.getMaxVel()); // Deal with
        // reverse
        m_turnController.setOutputConstraints(-10, 10);
        
        // Get error from end point
        SimPoint error = Robot.m_localizationEngine.getRotatedError(end.getTheta(), end.getX(), end.getY());
        double targetHeading;

        // Flip X if we are driving backwards
        if (error.getY() < 0) {
            error.setX(-error.getX());
        }

        // Increase turning aggression based on path progress
        double turnOffset = (error.getX() * turnRate);

        // Bind the turnOffset to the maximum turn rate
        turnOffset = Math.max(-constraints.getMaxTurn(), turnOffset);
        turnOffset = Math.min(constraints.getMaxTurn(), turnOffset);

        // Calculate target heading
        targetHeading = end.getTheta() - turnOffset;

        // Get gyroscope angle
        // double angle = Gyroscope.getInstance().getGyro().getAngle();
        // double angle = Gyroscope.getInstance().getFusedAngle();
        double angle = Robot.m_localizationEngine.getAngle();

        // Set setpoint for robot rotation
        m_turnController.setSetpoint(targetHeading);

        // Calculate Y's PID value
        double yOutput = m_forwardController.calculate(error.getY());

        // Calculate heading error
        double headingError = Math.abs(targetHeading - angle);
        System.out.println("HD_ERR_PRELIMIT: "+headingError);
        headingError = Math.min(headingError, 90);

        // Determine robot movement values
        double speed = yOutput * (((-1 * headingError) / 90.0) + 1); // 
        double rotation = -m_turnController.feed(angle);

        System.out.println("SPD: " + speed);
        System.out.println("RTA: "+ rotation );

        // Calculate motor speeds from arcade-style inputs
        // DriveSignal signal = DriveSignal.fromArcadeInputs(speed, rotation, DriveType.VELOCITY);

        // // Follow the signal
        // rawDrive(signal);

        m_differentialDrive.arcadeDrive(speed, rotation);

        // Check if the point has been reached
        double distance = error.getY();
        boolean finished = false;
        System.out.println("DIST:" +distance);

        System.out.println("ERR: " + error);
        System.out.println("HD_ERR: "+ headingError);

        // Check if the PID range is within epsilon
        if (constraints.getMinVel() <= 0.5) {
            if (m_forwardController.isFinished(epsilon)) { // Bug here
                stop();
                finished = true;
                System.out.println("Finished from epsilon");

            }
        } else if (Math.abs(distance) < epsilon) {
            finished = true;
            System.out.println("Finished from distance");
        }

        return finished;
    }

    /**
     * Turn to an angle
     * 
     * @param angle       Desired angle
     * @param constraints Drivetrain constraints
     * @param epsilon     Error for angle
     * 
     * @return Has the action finished?
     */
    public boolean turnTo(double angle, DriveConstraints constraints, double epsilon) {
        // Get turn output
        double rotation = -m_turnController.feed(Gyroscope.getInstance().getGyro().getAngle());

        // Bind output
        rotation = Math.max(rotation, -constraints.getMaxTurn());
        rotation = Math.min(rotation, constraints.getMaxTurn());

        // Determine if the action has finished
        if (m_turnController.isFinished(epsilon)) {
            // PID finished
            stop();
            return true;
        }

        // Send drive signal
        DriveSignal signal = DriveSignal.fromArcadeInputs(0.0, rotation, DriveType.STANDARD);
        rawDrive(signal);

        return false;
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
        // if (rotation == 0.0 && Math.abs(speed) > 0.05) {
        // double current_angle = Gyroscope.getInstance().getGyro().getAngle();

        // // Set drift correction and reset setpoint if this state is new
        // if (!m_driftCorrectionActive) {

        // m_driveCorrector.reset();
        // m_driveCorrector.setSetpoint(current_angle);

        // // Set state
        // m_driftCorrectionActive = true;
        // }

        // // Determine rotation correction
        // rotation = m_driveCorrector.feed(current_angle);
        // } else {
        // // Disable drift correction
        // m_driftCorrectionActive = false;
        // }

        // Handle inverse control
        speed = (invertControl) ? speed * -1 : speed;

        // Slew robot speed
        speed = m_speedSlew.feed(speed);

        // Set moving and turning trackers
        m_isMoving = (speed != 0.0);
        m_isTurning = (rotation != 0.0);

        // Feed drive command
        // m_differentialDrive.curvatureDrive(speed, rotation, quickTurn);
        m_differentialDrive.arcadeDrive(speed, rotation);

    }
    // double rightSquaredAccel = (rightMPS - lastRightMPS)

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

    public boolean getBrakes() {
        return m_desiredBrakeMode == NeutralMode.Brake;
    }

    // public void setMode(ControlType type) {
    // m_currentControlType = type;
    // }

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

    public void rawDrive(DriveSignal signal) {
        rawDrive(signal.getL(), signal.getR());
    }

    /**
     * Get the number of ticks recorded by the left GearBox's encoder.
     * 
     * @return Number of ticks
     */
    public int getLeftGearboxTicks() {
        // return m_leftGearbox.getTicks() - m_leftEncoderOffset;
        return m_leftEncoder.getTicks();
    }

    /**
     * Get the number of ticks recorded by the right GearBox's encoder.
     * 
     * @return Number of ticks
     */
    public int getRightGearboxTicks() {
        // return m_rightGearbox.getTicks() - m_rightEncoderOffset;
        return m_rightEncoder.getTicks();
    }

    /**
     * Get the distance traveled in meters by the left gearbox
     * 
     * @return Distance in meters
     */
    public double getLeftGearboxMeters() {
        // return ((getLeftGearboxTicks() / Constants.DriveTrain.ticksPerRotation) *
        // Constants.Robot.wheelCirc) / 100.0;
        return m_leftEncoder.getMeters(Constants.DriveTrain.ticksPerRotation, Constants.Robot.wheelCirc);
    }

    /**
     * Get the distance traveled in meters by the right gearbox
     * 
     * @return Distance in meters
     */
    public double getRightGearboxMeters() {
        // return ((getRightGearboxTicks() / Constants.DriveTrain.ticksPerRotation) *
        // Constants.Robot.wheelCirc) / 100.0;
        return m_rightEncoder.getMeters(Constants.DriveTrain.ticksPerRotation, Constants.Robot.wheelCirc);
    }

    public EncoderBase getLeftEncoder() {
        return m_leftEncoder;
    }

    public EncoderBase getRightEncoder() {
        return m_rightEncoder;
    }

    public void outputTelemetry() {

        SmartDashboard.putNumber("[DriveTrain] Left gearbox sensor", getLeftGearboxTicks());
        SmartDashboard.putNumber("[DriveTrain] Right gearbox sensor", getRightGearboxTicks());

    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveControl());

    }

    /**
     * To be called when the Drivetrain stops an action (path planning, or PID
     * movement)
     */
    public void stop() {
        rawDrive(0, 0);
        m_forwardController.reset();
        m_turnController.reset();
    }

    /**
     * Reset encoder readings to zero
     */
    public void zeroEncoders() {
        // m_leftEncoderOffset = m_leftGearbox.getTicks();
        // m_rightEncoderOffset = m_rightGearbox.getTicks();

        m_leftEncoder.zero();
        m_rightEncoder.zero();

    }

}