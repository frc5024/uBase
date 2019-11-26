package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib5k.components.Limelight.LEDMode;
import frc.lib5k.control.CubicDeadband;
import frc.lib5k.control.Toggle;
import frc.lib5k.utils.RobotLogger;
import frc.lib5k.utils.RobotLogger.Level;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveControl extends Command {
    RobotLogger logger = RobotLogger.getInstance();

    // Stored movement data
    double speed, rotation = 0.0;
    Toggle m_dtInvertToggle, m_rotationLimitToggle, m_cameraModeToggle;
    boolean m_shouldInvertControl = false;

    // Deadband
    CubicDeadband m_speedDeadband;
    CubicDeadband m_rotationDeadband;

    @SuppressWarnings("checkstyle:JavadocMethod")
    public DriveControl() {
        requires(Robot.m_drive);

        logger.log("[DriveControl] Configuring Deadband", Level.kRobot);
        m_speedDeadband = new CubicDeadband(0.0, Constants.Deadbands.speed_percision);
        m_rotationDeadband = new CubicDeadband(Constants.Deadbands.rotation_deadband,
                Constants.Deadbands.roataion_percision);

        m_dtInvertToggle = new Toggle();
        m_rotationLimitToggle = new Toggle();
        m_cameraModeToggle = new Toggle();
        m_rotationLimitToggle.feed(true);
    }

    @Override
    protected void initialize() {
        // Robot.m_drive.setMode(Drive.ControlType.DEFAULT);
    }

    @Override
    protected void execute() {

        // Read movement controls from driver
        speed = Robot.m_oi.getThrottle();
        rotation = Robot.m_oi.getTurn();

        // Read movement inversion
        m_shouldInvertControl = m_dtInvertToggle.feed(Robot.m_oi.getDriveTrainInvert());
        boolean quickTurn = m_rotationLimitToggle.feed(Robot.m_oi.getQuickTurn());
        boolean enablevision = m_cameraModeToggle.feed(Robot.m_oi.getCameraToggle());
        // boolean quickTurn = Robot.m_oi.getQuickTurn();

        // Pass data through deadbands
        // speed = m_speedDeadband.feed(speed);
        // rotation = m_rotationDeadband.feed(rotation);

        // Limit rotation
        rotation = (quickTurn) ? rotation : rotation * .9;

        // Send movement speeds to DriveTrain
        // Robot.m_drive.smoothDrive(speed, rotation, quickTurn, m_shouldInvertControl);
        Robot.m_drive.hybridDrive(speed, rotation, m_shouldInvertControl);
        Robot.m_limelight.setCamMode((enablevision) ? 0 : 1);
        Robot.m_limelight.setLED((enablevision) ? LEDMode.ON : LEDMode.OFF);

    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}