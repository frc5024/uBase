package frc.robot.paths;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import wpi2020.geometry.Pose2d;
import wpi2020.geometry.Rotation2d;
import wpi2020.geometry.Translation2d;
import wpi2020.trajectory.TrajectoryConfig;
// import wpi2020.trajectory.TrajectoryGenerator;

// public class WPIForward extends CommandGroup {

//     public WPIForward() {
//         /* Define path */

//         Pose2d startPoint = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
//         Pose2d endPoint = new Pose2d(0.0, 1.0, new Rotation2d(0.0));

//         // RObot max: 4.2672, 3.6576
//         TrajectoryConfig config = new TrajectoryConfig(3.0, 2.0);
//         config.setReversed(false);

//         wpi2020.trajectory.Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPoint,
//                 new ArrayList<Translation2d>(), endPoint, config);

//         // Set robot position
//         addSequential(new InstantCommand(() -> {
//             Robot.m_drive.resetRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
//         }));
//     }
// }