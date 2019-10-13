package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib5k.kinematics.FieldPosition;
import frc.robot.Robot;
import frc.robot.subsystems.Gyroscope;

public class SetRobotLocation extends Command {
    FieldPosition pos;

    public SetRobotLocation(FieldPosition pos) {
        this.pos = pos;
    }

    @Override
    protected void initialize() {
        Robot.m_localizationEngine.setXPos(pos.getX());
        Robot.m_localizationEngine.setYPos(pos.getY());
        Gyroscope.getInstance().overrideAutonOffset(pos.getTheta());        
    }
    
    @Override
    protected boolean isFinished() {
        return true;
    }
}