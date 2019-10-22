package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.lib5k.kinematics.DriveConstraints;
import frc.lib5k.kinematics.FieldPosition;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveToPoint extends TimedCommand {
    private FieldPosition position;
    private DriveConstraints constraints;
    private double turnRate, epsilon;
    private boolean finished = false;

    public DriveToPoint(FieldPosition point, double epsilon, double timeout) {
        this(point, Constants.Robot.robotConstraints, epsilon, timeout);
    }

    public DriveToPoint(FieldPosition point, DriveConstraints constraints, double epsilon, double timeout) {
        this(point, constraints, Constants.pathing_p, epsilon, timeout);
    }

    public DriveToPoint(FieldPosition point, DriveConstraints constraints, double turnRate, double epsilon,
            double timeout) {
        super(timeout);

        this.position = point;
        this.constraints = constraints;
        this.turnRate = turnRate;
        this.epsilon = epsilon;


        requires(Robot.m_drive);

    }

    @Override
    protected void initialize() {
        finished = false;
        System.out.println("Starting movement");
    }

    @Override
    protected void execute() {
        finished = Robot.m_drive.driveTo(position, constraints, turnRate, epsilon);

    }

    @Override
    protected boolean isFinished() {
        if (finished) {
            System.out.println("Finish movement");
        }
        return isTimedOut() || finished;
    }

    @Override
    protected void end() {
        boolean brakes = Robot.m_drive.getBrakes();
        Robot.m_drive.setBrakes(false);
        Robot.m_drive.stop();
        Robot.m_drive.setBrakes(brakes);
    }

}