package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.lib5k.kinematics.DriveConstraints;
import frc.robot.Constants;
import frc.robot.Robot;

//TODO: Waiting on DriveTrain tests

// public class TurnTo extends TimedCommand {
//     double desiredAngle, epsilon;
//     DriveConstraints constraints;
//     boolean finished = false;

//     public TurnTo(double angle, double epsilon, double timeout) {
//         this(angle, Constants.Robot.robotConstraints, epsilon, timeout);
//     }

//     public TurnTo(double angle, DriveConstraints constraints, double epsilon, double timeout) {
//         super(timeout);

//         this.desiredAngle = angle;
//         this.constraints = constraints;
//         this.epsilon = epsilon;

//     }

//     @Override
//     public void initSendable(SendableBuilder builder) {
//         finished = false;
//     }

//     @Override
//     protected void execute() {

//         finished = Robot.m_drive.turnTo(desiredAngle, constraints, epsilon);

//     }

//     @Override
//     protected boolean isFinished() {
//         return isTimedOut() || finished;
//     }
// }