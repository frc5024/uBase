package frc.robot.subsystems;

import frc.robot.subsystems.grip.GripSystem.GripState;

public class SuperstructurePose {

    public boolean armExtended;
    public double carriageAngle, wristAngle, rollerVoltage;
    public GripState gripState;

    public SuperstructurePose(boolean armExtended, double carriageAngle, double wristAngle, double rollerVoltage,
            GripState gripState) {
        this.armExtended = armExtended;
        this.carriageAngle = carriageAngle;
        this.wristAngle = wristAngle;
        this.rollerVoltage = rollerVoltage;
        this.gripState = gripState;

    }

    public SuperstructurePose(SuperstructurePose other) {
        this.armExtended = other.armExtended;
        this.carriageAngle = other.carriageAngle;
        this.wristAngle = other.wristAngle;
        this.rollerVoltage = other.rollerVoltage;
        this.gripState = other.gripState;
    }

    public void setFrom(SuperstructurePose other) {
        this.armExtended = other.armExtended;
        this.carriageAngle = other.carriageAngle;
        this.wristAngle = other.wristAngle;
        this.rollerVoltage = other.rollerVoltage;
        this.gripState = other.gripState;

    }

    public boolean isWristOverBumpers() {
        return false;
    }

}