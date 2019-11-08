package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib5k.kinematics.Error2D;
import frc.lib5k.kinematics.FieldPosition;
import frc.lib5k.kinematics.motionprofiling.MotionConstraints;
import frc.lib5k.kinematics.motionprofiling.MotionProfile;
import frc.lib5k.kinematics.motionprofiling.Motionprofiler;
import frc.lib5k.kinematics.motionprofiling.Motionprofiler.MotionOutput;
import frc.lib5k.utils.RobotLogger;
import frc.robot.Constants;
import frc.robot.Robot;
import jaci.pathfinder.Waypoint;

public class ProfileTo extends Command {
    RobotLogger logger = RobotLogger.getInstance();

    private FieldPosition position;
    private MotionConstraints constraints;
    private boolean finished = false;
    private MotionProfile profile;
    private Motionprofiler profiler;

    public ProfileTo(FieldPosition point) {
        this(point, Constants.Robot.robotMotionConstraints);
    }

    public ProfileTo(FieldPosition point, MotionConstraints constraints) {

        this.position = point;
        this.constraints = constraints;

        // Find error of the point from 0,0
        Error2D error = (new FieldPosition(0.0, 0.0)).getRotatedError(point);
        boolean is_reversed = error.getY() < 0.0;

        // Build a motionprofile from 0,0 to the specified point
        profile = new MotionProfile(constraints, is_reversed, new Waypoint(0, 0, 0), point.toWaypoint());

        // Configure the profiler
        profiler = new Motionprofiler(profile.generate(0.02), Constants.DriveTrain.motionProfilePID,
                Robot.m_drive.getLeftEncoder(), Robot.m_drive.getRightEncoder(), Constants.DriveTrain.ticksPerRotation,
                Constants.Robot.wheelDiameter, Constants.Robot.wheelbaseWidth, Robot.m_gyro.getGyro()::getAngle);

        requires(Robot.m_drive);

    }

    @Override
    protected void initialize() {
        finished = false;
        logger.log("[ProfileTo] Starting to follow path to " + position.toString());

        // Start the profiler
        profiler.start();
    }

    @Override
    protected void execute() {
        // Read the motor data
        MotionOutput output = profiler.getOutput();

        // Set the path completion
        finished = output.finished;

        // Send signals to motors
        Robot.m_drive.rawDrive(output.signal);

    }

    @Override
    protected void interrupted() {
        end();
    }

    @Override
    protected void end() {
        // Stop the MotionProfiler
        profiler.stop();

        // Stop the drivetrain
        Robot.m_drive.stop();

    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

}