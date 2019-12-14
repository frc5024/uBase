package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib5k.components.RaiderDrive;
import frc.lib5k.components.gyroscopes.NavX;
import frc.lib5k.components.motors.TalonSRXCollection;
import frc.lib5k.components.sensors.EncoderBase;
import frc.lib5k.control.CubicDeadband;
import frc.lib5k.control.SlewLimiter;
import frc.lib5k.kinematics.DriveConstraints;
import frc.lib5k.kinematics.DriveSignal;
import frc.lib5k.kinematics.FieldPosition;
import frc.lib5k.kinematics.MovementPlanner;
import frc.lib5k.kinematics.MovementSegment;
import frc.lib5k.spatial.LocalizationEngine;
import frc.lib5k.utils.RobotLogger;
import frc.lib5k.utils.RobotLogger.Level;
import frc.robot.Constants;
import frc.robot.commands.DriveControl;

/**
 * Handles control of the drivebase, tracking the robot's position, and
 * autonomous movement
 */
public class Drive extends Subsystem {

    private static Drive m_instance = null;
    // public enum ControlType {
    // DEFAULT, ASSIST, AUTONOMOUS, PATH
    // }

    RobotLogger logger = RobotLogger.getInstance();

    TalonSRXCollection m_leftGearbox;
    TalonSRXCollection m_rightGearbox;

    DifferentialDrive m_differentialDrive;
    SlewLimiter m_speedSlew;

    RaiderDrive m_raiderDrive;

    boolean m_driftCorrectionActive = false;
    boolean m_isMoving, m_isTurning = false;
    boolean m_isNewConfigData = false;

    NeutralMode m_desiredBrakeMode = NeutralMode.Coast;

    // ControlType m_currentControlType = ControlType.DEFAULT;

    /* Autonomous controllers */
    MovementPlanner m_movementPlanner;
    LocalizationEngine m_localizationEngine;

    // Encoders
    EncoderBase m_leftEncoder;
    EncoderBase m_rightEncoder;

    // Drive speeds

    private Drive() {
        logger.log("Building drive", Level.kRobot);

        // Build drive gearboxes
        logger.log("DriveTrain", "Linking motors", Level.kRobot);
        m_leftGearbox = new TalonSRXCollection(new WPI_TalonSRX(Constants.DriveTrain.leftFrontMotor),
                new WPI_TalonSRX(Constants.DriveTrain.leftRearMotor));
        m_rightGearbox = new TalonSRXCollection(new WPI_TalonSRX(Constants.DriveTrain.rightFrontMotor),
                new WPI_TalonSRX(Constants.DriveTrain.rightRearMotor));

        // Config current limits
        logger.log("DriveTrain", "Configuring current limiting for gearboxes");
        m_leftGearbox.setCurrentLimit(Constants.DriveTrain.peakCurrent, Constants.DriveTrain.currentTimeout,
                Constants.DriveTrain.holdCurrent, 0);
        m_rightGearbox.setCurrentLimit(Constants.DriveTrain.peakCurrent, Constants.DriveTrain.currentTimeout,
                Constants.DriveTrain.holdCurrent, 0);

        // Build drivebase
        logger.log("DriveTrain", "Building differential drivebase controller", Level.kRobot);
        m_differentialDrive = new DifferentialDrive(m_leftGearbox, m_rightGearbox);
        m_differentialDrive.setSafetyEnabled(false);
        m_leftGearbox.setMasterMotorSafety(false);
        m_rightGearbox.setMasterMotorSafety(false);

        // Set up RaiderDrive
        m_raiderDrive = new RaiderDrive(new CubicDeadband(0.0, Constants.Deadbands.speed_percision),
                new CubicDeadband(Constants.Deadbands.rotation_deadband, Constants.Deadbands.roataion_percision));
        m_raiderDrive.setRampRate(Constants.accelerationStep);

        m_differentialDrive.setDeadband(0.02);

        // Configure Slew limiter
        m_speedSlew = new SlewLimiter(Constants.accelerationStep);

        // Configure encoders
        m_leftEncoder = m_leftGearbox.getEncoder(0);
        m_rightEncoder = m_rightGearbox.getEncoder(0);

        // Configure Autonomous controllers
        m_movementPlanner = new MovementPlanner(Constants.DriveTrain.forwardPIDGains,
                Constants.DriveTrain.turnPIDGains);
        m_localizationEngine = LocalizationEngine.getInstance();

        // Publish MovementPlanner PIDControllers
        m_movementPlanner.publishPIDControllers();

        setRampRate(0.12);

    }

    public static Drive getInstance() {
        if (m_instance == null) {
            m_instance = new Drive();
        }

        return m_instance;
    }

    @Override
    public void periodic() {
        // Update encoders
        m_leftEncoder.update();
        m_rightEncoder.update();

        // Handle talon config data
        if (m_isNewConfigData) {

            // Set brake mode for all talons
            m_leftGearbox.getMaster().setNeutralMode(m_desiredBrakeMode);
            m_rightGearbox.getMaster().setNeutralMode(m_desiredBrakeMode);

            // Data had been sent, disable lock
            m_isNewConfigData = false;
        }

        // Read encoder positions
        double leftMeters = m_leftEncoder.getMeters(Constants.DriveTrain.ticksPerRotation, Constants.Robot.wheelCirc);
        double rightMeters = m_rightEncoder.getMeters(Constants.DriveTrain.ticksPerRotation, Constants.Robot.wheelCirc);

        // Read robot heading
        double heading = NavX.getInstance().getAngle();

        // Update the LocalizationEngine
        m_localizationEngine.calculate(leftMeters, rightMeters, heading);

        // Output telemetry data
        outputTelemetry();

        // System.out.println(String.format("L: %.2f | R: %.2f",
        // m_leftGearbox.getEstimatedVoltage(), m_rightGearbox.getEstimatedVoltage()));

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

        // Read the MovementPlanner's MovementSegment
        MovementSegment segment = m_movementPlanner.compute(end, constraints, turnRate, epsilon);

        // System.out.println(segment);

        // Send segment data to motors
        arcadeDrive(segment.getSpeed(), segment.getTurn());

        // Return weather or not the segment is finished (has the robot reached the end
        // point)
        return segment.isFinished();
    }

    /**
     * Drive the robot with some help from sensors
     * 
     * @param speed     Desired robot speed
     * @param rotation  Desired robot turn rate
     * @param quickTurn Should quickTurn be enabled?
     */
    public void smoothDrive(double speed, double rotation, boolean quickTurn, boolean invertControl) {

        // Handle inverse control
        speed = (invertControl) ? speed * -1 : speed;

        // Slew robot speed
        speed = m_speedSlew.feed(speed);

        // Set moving and turning trackers
        m_isMoving = (speed != 0.0);
        m_isTurning = (rotation != 0.0);

        // Feed drive command
        // m_differentialDrive.curvatureDrive(speed, rotation, quickTurn);
        m_differentialDrive.arcadeDrive(speed, rotation, true);

    }

    public void hybridDrive(double speed, double turn, boolean invertControl) {
        // Determine DriveSignal
        DriveSignal signal = m_raiderDrive.computeSemiConst(speed * ((invertControl) ? -1 : 1), turn, true);

        rawDrive(signal);

    }

    public void arcadeDrive(double speed, double rotation) {
        // System.out.println("" + speed + ", " + rotation);
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

        // This needs to be negated
        m_rightGearbox.set(-r);
    }

    public void rawDrive(DriveSignal signal) {
        rawDrive(signal.getL(), signal.getR());
    }

    public EncoderBase getLeftEncoder() {
        return m_leftEncoder;
    }

    public EncoderBase getRightEncoder() {
        return m_rightEncoder;
    }

    public void setRampRate(double rate) {
        m_leftGearbox.getMaster().configOpenloopRamp(rate);
        m_rightGearbox.getMaster().configOpenloopRamp(rate);
    }

    public void outputTelemetry() {

        SmartDashboard.putNumber("[DriveTrain] Left gearbox sensor", getLeftEncoder().getRawTicks());
        SmartDashboard.putNumber("[DriveTrain] Right gearbox sensor", getRightEncoder().getRawTicks());

        m_leftGearbox.updateTelemetry();
        m_rightGearbox.updateTelemetry();

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
        m_raiderDrive.reset();
    }

    /**
     * Reset encoder readings to zero
     */
    public void zeroEncoders() {

        m_leftEncoder.zero();
        m_rightEncoder.zero();

    }

}