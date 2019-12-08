package frc.lib5k.testing;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib5k.interfaces.Loggable;
import frc.lib5k.testing.interfaces.IShuffleboardTestable;
import frc.lib5k.utils.RobotLogger;
import frc.lib5k.utils.telemetry.ComponentTelemetry;

public class ShuffleboardTester implements Loggable {
    RobotLogger logger = RobotLogger.getInstance();
    private static ShuffleboardTester m_instance = null;

    /* Telemetry */
    private String name;
    private NetworkTable telemetryTable;

    /* Status */
    private boolean enabled;

    /* Components */
    private ArrayList<IShuffleboardTestable> components = new ArrayList<>();

    /* Modes */
    private SendableChooser<String> modeChooser;

    private ShuffleboardTester() {

        // Build the mode chooser
        modeChooser = new SendableChooser<>();
        modeChooser.setDefaultOption("Component tests", "component");
        modeChooser.addOption("Motor tester", "motortester");

        // Set component name
        name = "ShuffleboardTester";

        // Get the telemetry networktable
        telemetryTable = ComponentTelemetry.getInstance().getTableForComponent(name);

    }

    public static ShuffleboardTester getInstance() {
        if (m_instance == null) {
            m_instance = new ShuffleboardTester();
        }

        return m_instance;
    }

    /**
     * Publish the test mode selector. This is just a chooser that lets the user
     * choose between "Motor tester" and "Component tests"
     */
    public void publish() {

        // Publish the mode chooser
        Shuffleboard.getTab("ShuffleboardTester").add("Test mode chooser", modeChooser);

    }

    public void enable() {
        enabled = true;

        // Publish the correct tab for testing
        switch (modeChooser.getSelected()) {
        case "component":
            publishComponentTests();
            break;
        case "motortester":
            publishMotorTester();
            break;
        }

    }

    public void disable() {
        enabled = false;

    }

    private void publishComponentTests() {

    }

    private void publishMotorTester() {

    }

    public void register(IShuffleboardTestable component) {
        components.add(component);

    }

    @Override
    public void logStatus() {
        // TODO Auto-generated method stub

    }

    @Override
    public void updateTelemetry() {
        telemetryTable.getEntry("Enabled").setBoolean(enabled);

    }
}