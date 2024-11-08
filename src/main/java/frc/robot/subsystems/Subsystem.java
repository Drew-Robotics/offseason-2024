package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {
    protected abstract void dashboardInit();
    protected abstract void dashboardPeriodic();

    protected abstract void publishInit();
    protected abstract void publishPeriodic();

    protected Subsystem(String name) {
        dashboardInit();
    }

    @Override
    public void periodic() {
        super.periodic();
        publishPeriodic();
        dashboardPeriodic();
    }
}
