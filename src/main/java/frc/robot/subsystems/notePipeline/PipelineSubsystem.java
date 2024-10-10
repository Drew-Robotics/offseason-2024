package frc.robot.subsystems.notePipeline;

import frc.robot.subsystems.Subsystem;

public abstract class PipelineSubsystem extends Subsystem {
    public PipelineSubsystem(String name) {
        super(name);
    }

    public abstract void run(double mps);
    public void stop() { run(0); }

    @Override
    protected void dashboardInit() {}

    @Override
    protected void publishInit() {}

    @Override
    protected void publishPeriodic() {}
}
