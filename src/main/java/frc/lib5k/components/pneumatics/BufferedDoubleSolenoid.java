package frc.lib5k.components.pneumatics;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib5k.interfaces.IFlushable;
import frc.lib5k.interfaces.Loggable;
import frc.lib5k.utils.RobotLogger;
import frc.lib5k.utils.telemetry.ComponentTelemetry;

public class BufferedDoubleSolenoid extends DoubleSolenoid implements IFlushable, Loggable {
    RobotLogger logger = RobotLogger.getInstance();

    /* Locals */
    private Value lastValue = null;

    /* Telemetry */
    private String name;
    private NetworkTable telemetryTable;

    public BufferedDoubleSolenoid(int moduleNumber, int forwardChannel, int reverseChannel) {
        super(moduleNumber, forwardChannel, reverseChannel);

        // Determine component name
        name = String.format("BufferedDoubleSolenoid (%d:%d+%d)", moduleNumber, forwardChannel, reverseChannel);

        // Get telemetry table
        telemetryTable = ComponentTelemetry.getInstance().getTableForComponent(name);
    }

    @Override
    public void set(Value value) {

        // Check if there is a new command
        if (value != lastValue) {

            // Set solenoid mode
            super.set(value);

            // Set last state
            lastValue = value;
        }
    }

    /**
     * re-send the current state to flush CAN
     */
    @Override
    public void flush() {
        super.set(lastValue);

    }

    @Override
    public void logStatus() {

        // Build status string
        String status = String.format("Value: %s", lastValue.toString());

        // Log status
        logger.log(name, status);

    }

    @Override
    public void updateTelemetry() {
        telemetryTable.getEntry("State").setString(lastValue.toString());

    }

}