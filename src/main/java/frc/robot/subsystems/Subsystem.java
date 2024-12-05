package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {
    protected final String m_name;
    protected final NetworkTable m_table;

    protected abstract void dashboardInit();
    protected abstract void dashboardPeriodic();

    protected abstract void publishInit();
    protected abstract void publishPeriodic();

    protected Subsystem(String name) {
        m_name = name;
        m_table = NetworkTableInstance.getDefault().getTable(m_name);
        publishInit();
        dashboardInit();
    }

    @Override
    public void periodic() {
        super.periodic();
        publishPeriodic();
        dashboardPeriodic();
    }

}
