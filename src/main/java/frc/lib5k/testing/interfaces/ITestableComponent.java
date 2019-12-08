package frc.lib5k.testing.interfaces;

public interface ITestableComponent {
    public void onTestStart();

    public void testPeriodic();

    public void onTestEnd();
}