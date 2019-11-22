package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // Controllers
  public XboxController driverController = new XboxController(0);

  /**
   * Get the DriveTrain throttle value from driverstation
   * 
   * @return Throttle (from -1.0 to 1.0)
   */
  public double getThrottle() {
    double speed = 0.0;

    speed += driverController.getTriggerAxis(GenericHID.Hand.kRight);
    speed -= driverController.getTriggerAxis(GenericHID.Hand.kLeft);

    return speed;
  }

  /**
   * Get the DriveTrain turn rate value from driverstation
   * 
   * @return Turn rate (from -1.0 to 1.0)
   */
  public double getTurn() {
    return driverController.getX(GenericHID.Hand.kLeft);
  }

  /**
   * Should the bot flip it's orientation (toggle input)
   * 
   * @return Output
   */
  public boolean getDriveTrainInvert() {
    return driverController.getXButtonPressed();
  }

  public boolean getQuickTurn() {
    return driverController.getAButtonPressed();
  }

  public boolean getCameraToggle() {
    return driverController.getBButtonPressed();
  }

}
