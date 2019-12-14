package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;

public class DriveTrainController {

    // DriveTrain object for use by commands
    Drive drive = Robot.m_drive;
    
    public class TurnTo extends Command {

        @Override
        protected boolean isFinished() {
            // TODO Auto-generated method stub
            return false;
        }
        
    }
}