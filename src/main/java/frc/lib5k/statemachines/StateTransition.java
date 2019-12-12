package frc.lib5k.statemachines;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * A wrapper for CommandGroup that allows us to specify if the group can even
 * run in the first place
 */
public abstract class StateTransition extends CommandGroup {

    public abstract boolean canRun();

    public abstract boolean hasCompleted();

    @Override
    protected boolean isFinished() {
        return !canRun() || hasCompleted() || super.isFinished();
    }
}