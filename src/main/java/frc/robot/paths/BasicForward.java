package frc.robot.paths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib5k.kinematics.DriveConstraints;
import frc.lib5k.kinematics.FieldPosition;
import frc.robot.autonomous.actions.DriveToPoint;
import frc.robot.autonomous.actions.SetRobotLocation;

public class BasicForward extends CommandGroup {
    public BasicForward() {
        addSequential(new SetRobotLocation(new FieldPosition(0, 0, 0)));
        addSequential(new DriveToPoint(new FieldPosition(0, 2, 0), new DriveConstraints(0, 2.0), 3.0, 0.2, 15.0));
    }
}