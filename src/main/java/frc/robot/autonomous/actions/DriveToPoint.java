package frc.robot.autonomous.actions;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.lib5k.kinematics.DriveConstraints;
import frc.lib5k.kinematics.FieldPosition;
import frc.lib5k.utils.RobotLogger;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveToPoint extends TimedCommand {
    RobotLogger logger = RobotLogger.getInstance();

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
        logger.log("[DriveToPoint] Starting to follow path to " + position.toString() + " with a max turn rate of "
                + constraints.getMaxTurn() + " and an epsilon of " + epsilon);
    }

    @Override
    protected void execute() {
        finished = Robot.m_drive.driveTo(position, constraints, turnRate, epsilon);

    }

    @Override
    protected boolean isFinished() {
        if (finished) {
            logger.log("[DriveToPoint] Reached point");
        }
        return isTimedOut() || finished;
    }

    @Override
    protected void end() {
        Robot.m_drive.stop();
    }

}