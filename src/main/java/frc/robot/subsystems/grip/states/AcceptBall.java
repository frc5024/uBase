package frc.robot.subsystems.grip.states;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.lib5k.statemachines.StateTransition;
import frc.robot.subsystems.grip.GripSystem;
import frc.robot.subsystems.grip.actions.PullUntilBallCollected;

public class AcceptBall extends StateTransition {
    public AcceptBall() {

        // Switch the gripper to the wheel side
        addSequential(new InstantCommand(() -> {
            GripSystem.getInstance().setGrip(false);
        }));

        // Intake the ball
        addSequential(new PullUntilBallCollected());

        // Grip the ball with the grippers
        addSequential(new InstantCommand(() -> {
            GripSystem.getInstance().setGrip(true);
        }));

    }

    @Override
    public boolean canRun() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean hasCompleted() {

        // This can stay false. StateTransition will automatically finish when all
        // commands are finished
        return false;
    }
}