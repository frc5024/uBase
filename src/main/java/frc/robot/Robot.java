package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.lib5k.components.USBVisionCamera;
import frc.lib5k.components.limelight.Limelight;
import frc.lib5k.components.limelight.Limelight.LEDMode;
import frc.lib5k.roborio.FaultReporter;
import frc.lib5k.simulation.Hooks;
import frc.lib5k.utils.RobotLogger;
import frc.lib5k.utils.RobotLogger.Level;
import frc.robot.autonomous.Chooser;
import frc.robot.commands.DriveControl;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Gyroscope;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	RobotLogger logger = RobotLogger.getInstance();
	FaultReporter reporter = FaultReporter.getInstance();

	/* Subsystems */
	public static Drive m_drive = new Drive();
	public static OI m_oi;
	public static Gyroscope m_gyro = Gyroscope.getInstance();
	public static Limelight m_limelight = new Limelight();

	/* Commands */
	DriveControl m_driveControl;

	/* Auton */
	Chooser m_chooser;

	CommandGroup m_autonomousCommand;

    USBVisionCamera m_camera;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		logger.log("Robot initializing", Level.kRobot);
		m_oi = new OI();

		logger.log("Building commands", Level.kRobot);
		m_driveControl = new DriveControl();

		m_chooser = new Chooser();

		logger.start(0.02);

		// Init sensors
		Gyroscope.getInstance().getGyro().reset();
		Gyroscope.getInstance().setAutonOffset();
		m_drive.zeroEncoders();

		// Turn off Limelight LEDs
		m_limelight.setLEDMode(LEDMode.OFF);

		// Connect camera
		// m_camera = new USBVisionCamera("Main camera", 0,8, Constants.pcm_led);
		// m_camera.keepCameraAwake(true);

		// m_camera.setLED(USBVisionCamera.LEDMode.BLINK);

		// Run Teleop in simulation
		Hooks.setStateIfSimulated(Hooks.RobotState.TELEOP);

	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		if (isSimulation()) {
			m_drive.hybridDrive(.5, .2, false);
		}

		

	}

	private void sharedInit() {
		// Reduce network stress by disabling default telem. If LiveWindow is needed,
		// reboot bot, then start Test Mode
		LiveWindow.disableAllTelemetry();

		m_drive.setBrakes(true);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
		logger.log("Robot", "Disabled", Level.kRobot);
		m_drive.setBrakes(false);
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		logger.log("Robot", "Autonomous starting", Level.kRobot);
		sharedInit();

		// Set the autonomous gyro offset
		Gyroscope.getInstance().getGyro().reset();
		Gyroscope.getInstance().setAutonOffset();
		m_drive.zeroEncoders();
		m_drive.stop();

		// Read selected autonomous mode
		m_autonomousCommand = m_chooser.getAutonomousCommand();
		// m_autonomousCommand.start();

		// Try to start the command
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();

	}

	@Override
	public void teleopInit() {
		logger.log("Robot", "Teleop starting", Level.kRobot);
		sharedInit();

		// Stop autonomous
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		/* Start commands */
		if (m_driveControl != null) {
			m_driveControl.start();
		}

	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void testInit() {

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
