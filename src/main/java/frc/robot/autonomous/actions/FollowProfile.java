package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib5k.kinematics.motionprofiling.MotionProfile;
import frc.lib5k.kinematics.motionprofiling.Motionprofiler;
import frc.lib5k.kinematics.motionprofiling.Motionprofiler.MotionOutput;
import frc.robot.Constants;
import frc.robot.Robot;

public class FollowProfile extends Command {
    private boolean finished = false;
    private MotionProfile m_profile;
    private Motionprofiler m_profiler;

    public FollowProfile(MotionProfile profile) {
        this(profile, false);
    }

    public FollowProfile(MotionProfile profile, boolean pre_generate) {

        // Store profile, and generate if needed
        m_profile = (pre_generate) ? profile.generate(0.02) : profile;

        // Create the MotionProfiler
        m_profiler = new Motionprofiler(m_profile, Constants.DriveTrain.motionProfilePID,
                Robot.m_drive.getLeftEncoder(), Robot.m_drive.getRightEncoder(), Constants.DriveTrain.ticksPerRotation,
                Constants.Robot.wheelDiameter, Constants.Robot.wheelbaseWidth, Robot.m_gyro.getGyro()::getAngle);

        requires(Robot.m_drive);
    }

    @Override
    protected void initialize() {
        finished = false;
        
        // Start the MotionProfiler
        m_profiler.start();
    }

    @Override
    protected void execute() {
        // Read the motor data
        MotionOutput output = m_profiler.getOutput();

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
        m_profiler.stop();

        // Stop the drivetrain
        Robot.m_drive.stop();

    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

}