package frc.lib5k.framework;

/**
 * A common interface for classes that can handle robot state changes
 */
public interface RobotStateHandler {

    /**
     * To be run when the robot is enabled
     */
    public void onEnable();


    /**
     * To be run when the robot is disabled
     */
    public void onDisable();

}