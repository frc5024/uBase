package frc.robot.subsystems;

import frc.robot.subsystems.grip.GripSystem.GripState;

public class SuperstructureGoal extends SuperstructurePose {

    public SuperstructureGoal(boolean armExtended, double carriageAngle, double wristAngle, double rollerVoltage,
            GripState gripState) {
        super(armExtended, carriageAngle, wristAngle, rollerVoltage, gripState);
    }

    public boolean isAtPose(SuperstructurePose pose) {
        // return 
    }

}