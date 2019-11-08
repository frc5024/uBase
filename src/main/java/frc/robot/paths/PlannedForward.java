package frc.robot.paths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib5k.kinematics.FieldPosition;
import frc.lib5k.kinematics.motionprofiling.MotionConstraints;
import frc.robot.autonomous.actions.ProfileTo;

public class PlannedForward extends CommandGroup {
    
    public PlannedForward() {
        addSequential(new ProfileTo(new FieldPosition(0.0, 1.0, 0.0), new MotionConstraints(3.0, 1.0, 60)));
    }
}