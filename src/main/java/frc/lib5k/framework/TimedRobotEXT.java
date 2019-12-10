package frc.lib5k.framework;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * TimedRobot with a few extra features
 */
public class TimedRobotEXT extends TimedRobot {
    private ArrayList<RobotStateHandler> stateHandlers = new ArrayList<>();

    /**
     * Register a robot state handler
     * 
     * @param handler State handler
     */
    public void registerStateHandler(RobotStateHandler handler) {
        stateHandlers.add(handler);

    }

    /**
     * Un-register a robot state handler
     * 
     * @param handler State handler
     */
    public void unregisterStateHandler(RobotStateHandler handler) {
        stateHandlers.remove(handler);
    }

    /**
     * Call all RobotStateHandlers with the enable call
     */
    public void handleRobotEnable() {

        // Iter each handler
        for (RobotStateHandler handler : stateHandlers) {

            // Call the handler enable method
            handler.onEnable();
        }
    }

    /**
     * Call all RobotStateHandlers with the disable call
     */
    public void handleRobotDisable() {

        // Iter each handler
        for (RobotStateHandler handler : stateHandlers) {

            // Call the handler disable method
            handler.onDisable();
        }
    }

    public void sharedInit() {
        System.out.println("Default sharedInit() method called.. Overload me!");
    }
}