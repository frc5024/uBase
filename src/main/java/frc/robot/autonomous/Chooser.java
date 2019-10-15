package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.paths.BasicForward;

/**
 * Reads inputs from smartdashboard to determine correct auto to run
 */
public class Chooser {
    SendableChooser<Integer> m_positionChooser = new SendableChooser<Integer>();
    SendableChooser<Integer> m_targetChooser = new SendableChooser<Integer>();

    /**
     * Create a chooser and register with smartdashboard
     */
    public Chooser() {

        // Load starting positions
        m_positionChooser.setName("Robot position");
        m_positionChooser.setDefaultOption("Default", 0);

        // Load target positions
        m_targetChooser.setName("Autonomous target");
        m_targetChooser.setDefaultOption("Do nothing", 0);
        m_targetChooser.addOption("Forward 2 Meters", 10);

        // Push choosers to dashboard
        Shuffleboard.getTab("DriverStation").add(m_positionChooser);
        Shuffleboard.getTab("DriverStation").add(m_targetChooser);
    }

    public CommandGroup getAutonomousCommand() {
        // Calculate autonomous key from choosers
        int key = m_positionChooser.getSelected() + m_targetChooser.getSelected();
        System.out.println(key);

        // Detect commandgroup to use
        switch (key) {
        case 10:
            return new BasicForward();
        default:
            break;
        }
        return null;
    }

}