package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.team1114.SimPoint;

/**
 * Reads various sensor inputs to determine robot position.
 * 
 * To learn more about localization, see:
 * https://www.chiefdelphi.com/t/pin-point-robots-position-gps/67670/8
 */
public class LocalizationEngine extends Subsystem {

    private double xPos, yPos = 0.0;
    private double timeDelta = 20;
    private double lastTime = 0.0;

    private double lastLeftMPS, lastRightMPS = 0.0;

    AHRS m_gyroInstance;

    public LocalizationEngine() {
        m_gyroInstance = Gyroscope.getInstance().getGyro();
    }

    @Override
    protected void initDefaultCommand() {
    }

    @Override
    public void periodic() {
        // Calculate time
        timeDelta = System.currentTimeMillis() - lastTime;
        lastTime = System.currentTimeMillis();

        // Get the gyro angle, and account for offset
        // TODO: Not sure which one of these lines is correct
        double gyroAngle = m_gyroInstance.getAngle() + (Gyroscope.getInstance().getAutonOffset() - 90);
        // double gyroAngle = m_gyroInstance.getAngle() -
        // Gyroscope.getInstance().getAutonOffset();

        // Calculate M/S of each gearbox
        double leftMPS = Robot.m_drive.getLeftGearboxMeters() * (1000.0 / timeDelta);
        double rightMPS = Robot.m_drive.getRightGearboxMeters() * (1000.0 / timeDelta);

        // Calculate acceleration for each gearbox (squared)
        // double leftSquaredAccel = (leftMPS - lastLeftMPS) / (timeDelta / 1000.0);
        // double rightSquaredAccel = (rightMPS - lastRightMPS) / (timeDelta / 1000.0);

        // Set last MPS calcs
        lastLeftMPS = leftMPS;
        lastRightMPS = rightMPS;

        // Determine drive velocity
        double driveVelocityMPS = (leftMPS + rightMPS) / 2.0;

        // Determine robot XY speed
        double xSpeed = driveVelocityMPS * Math.cos(Math.toRadians(gyroAngle));
        double ySpeed = driveVelocityMPS * Math.sin(Math.toRadians(gyroAngle));

        // Set Robot position
        xPos += xSpeed * timeDelta / 1000.0;
        yPos += ySpeed * timeDelta / 1000.0;
    }

    public double getXPos() {
        return xPos;

    }

    public double getYPos() {
        return yPos;
    }

    /**
     * Rotates the xy coordinates to be relative to the angle of the target
     * 
     * From team 1114:
     * https://bitbucket.org/kaleb_dodd/simbot2019public/src/master/src/main/java/frc/subsystems/Drive.java
     */
    public SimPoint getRotatedError(double theta, double desiredX, double desiredY) {
        double currentX = getXPos();
        double currentY = getYPos();
        double rotation = 90 - theta;

        SimPoint currentPosition = new SimPoint(currentX, currentY);
        SimPoint finalPosition = new SimPoint(desiredX, desiredY);

        currentPosition.rotateByAngleDegrees(rotation);
        finalPosition.rotateByAngleDegrees(rotation);

        double xError = finalPosition.getX() - currentPosition.getX();
        double yError = finalPosition.getY() - currentPosition.getY();

        return new SimPoint(xError, yError);

    }

}