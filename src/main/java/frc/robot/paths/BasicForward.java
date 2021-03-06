package frc.robot.paths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib5k.kinematics.DriveConstraints;
import frc.lib5k.kinematics.FieldPosition;
import frc.robot.autonomous.actions.DriveToPoint;
import frc.robot.autonomous.actions.SetRobotLocation;

/**
 * REMEMBER to add 90 to every angle. This is field-relative
 */
public class BasicForward extends CommandGroup {

    public BasicForward() {
        addSequential(new SetRobotLocation(new FieldPosition(0, 0, 0)));
        // addSequential(new DriveToPoint(new FieldPosition(0, 2, 0), new DriveConstraints(0, 1.0, 0.1), 0.2, 0.2, 5.0));
        addSequential(new DriveToPoint(new FieldPosition(0, 0.1, 90), new DriveConstraints(0, 1.0, 0.5), 0.2, 0.0, 5.0));
    }
}