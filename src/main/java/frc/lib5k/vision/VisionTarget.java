package frc.lib5k.vision;

public interface VisionTarget {
    public double getSkew();

    public boolean isVisible();

    public double getLastSeenTimestamp();
}