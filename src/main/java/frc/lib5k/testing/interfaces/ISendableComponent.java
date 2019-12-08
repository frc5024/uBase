package frc.lib5k.testing.interfaces;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public interface ISendableComponent {
    public void publish(ShuffleboardTab tab);

    public ShuffleboardLayout buildLayout();
}