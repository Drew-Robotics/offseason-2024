package frc.robot.subsystems.notePipeline;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.NotePipelineConstants;
import frc.robot.Constants.NotePipelineConstants.CANIDs;

public class IntakeSubsystem extends PipelineSubsystem {
    private final CANSparkFlex m_intakeMotor;
    private final SparkPIDController m_intakePID;
    private final RelativeEncoder m_intakeEncoder;

    private static IntakeSubsystem m_instance;

    public static IntakeSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new IntakeSubsystem();
        return m_instance;
    }

    public IntakeSubsystem() {
        super(IntakeSubsystem.class.getSimpleName());

        m_intakeMotor = new CANSparkFlex(CANIDs.kIntake, MotorType.kBrushless);

        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_intakeEncoder.setPositionConversionFactor(NotePipelineConstants.kEncoderPositionFactor);
        m_intakeEncoder.setPositionConversionFactor(NotePipelineConstants.kEncoderVelocityFactor);

        m_intakePID = m_intakeMotor.getPIDController();
        m_intakePID.setFeedbackDevice(m_intakeEncoder);
        m_intakePID.setP(0.01);
        m_intakePID.setI(0);
        m_intakePID.setD(0);
    }

    public void set(double mps) {
        m_intakePID.setReference(mps, ControlType.kVelocity);
    }

    @Override
    protected void dashboardPeriodic() {
        SmartDashboard.putNumber("Intake Velocity", m_intakeEncoder.getVelocity());
    }
}
