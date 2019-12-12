package frc.robot.subsystems.grip.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.grip.GripSystem;

/**
 * Will spin up carriage collector motors until a ball is grabbed
 */
public class PullUntilBallCollected extends Command {
    GripSystem system = GripSystem.getInstance();

    @Override
    protected void execute() {

        // Set intake RPM to 1/3 max (in reverse)
        system.setWantedRPM((system.MAX_RPM / 3) * -1);

    }

    @Override
    protected void end() {

        // Stop the intake motors
        system.setWantedRPM(0);
    }

    @Override
    protected boolean isFinished() {

        // If we have a ball, we have succeeded
        return system.hasBall();
    }

}