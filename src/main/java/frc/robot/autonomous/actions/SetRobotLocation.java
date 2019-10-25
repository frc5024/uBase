package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib5k.kinematics.FieldPosition;
import frc.lib5k.spatial.LocalizationEngine;

public class SetRobotLocation extends Command {
    FieldPosition pos;

    public SetRobotLocation(FieldPosition pos) {
        this.pos = pos;
    }

    @Override
    protected void initialize() {
        LocalizationEngine.getInstance().setRobotPosition(pos);       
    }
    
    @Override
    protected boolean isFinished() {
        return true;
    }
}