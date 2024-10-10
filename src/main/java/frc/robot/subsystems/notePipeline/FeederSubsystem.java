package frc.robot.subsystems.notePipeline;

import com.revrobotics.CANSparkBase.ControlType;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.NotePipelineConstants.CANIDs;

public class FeederSubsystem extends PipelineSubsystem {
    private final CANSparkFlex m_feederMotor;
    private final SparkPIDController m_feederPID;
    private final RelativeEncoder m_feederEncoder;

    private static FeederSubsystem m_instance;

    public static FeederSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new FeederSubsystem();
        return m_instance;
    }

    public FeederSubsystem() {
        super(FeederSubsystem.class.getSimpleName());

        m_feederMotor = new CANSparkFlex(CANIDs.kFeeder, MotorType.kBrushless);
        m_feederEncoder = m_feederMotor.getEncoder();
        m_feederPID = m_feederMotor.getPIDController();
        m_feederPID.setFeedbackDevice(m_feederEncoder);
    }

    public void run(double mps) {
        m_feederPID.setReference(mps, ControlType.kVelocity);
    }

    @Override
    protected void dashboardPeriodic() {
        SmartDashboard.putNumber("Feeder Velocity", m_feederEncoder.getVelocity());
    }
}
