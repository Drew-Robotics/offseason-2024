package frc.robot.subsystems.notePipeline;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.NotePipelineConstants;
import frc.robot.Constants.NotePipelineConstants.CANIDs;

public class ShooterSubsystem extends PipelineSubsystem {
    private final CANSparkFlex m_shooterMotorL, m_shooterMotorR;
    private final SparkPIDController m_shooterPIDL, m_shooterPIDR;
    private final RelativeEncoder m_shooterEncoderL, m_shooterEncoderR;
    private double m_targetVelMPS = 0;

    private static ShooterSubsystem m_instance;

    public static ShooterSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new ShooterSubsystem();
        return m_instance;
    }

    public ShooterSubsystem() {
        super(ShooterSubsystem.class.getSimpleName());

        m_shooterMotorL = new CANSparkFlex(CANIDs.kShooterLeft, MotorType.kBrushless);

        m_shooterEncoderL = m_shooterMotorL.getEncoder();
        m_shooterEncoderL.setPositionConversionFactor(NotePipelineConstants.kEncoderPositionFactor);
        m_shooterEncoderL.setVelocityConversionFactor(NotePipelineConstants.kEncoderVelocityFactor);

        m_shooterPIDL = m_shooterMotorL.getPIDController();
        m_shooterPIDL.setFeedbackDevice(m_shooterEncoderL);
        m_shooterPIDL.setP(0.1);
        m_shooterPIDL.setI(0.0);
        m_shooterPIDL.setD(0.0);
        m_shooterPIDL.setFF(0.03); // 6784

        m_shooterMotorR = new CANSparkFlex(CANIDs.kShooterRight, MotorType.kBrushless);

        m_shooterEncoderR = m_shooterMotorR.getEncoder();
        m_shooterEncoderR.setPositionConversionFactor(NotePipelineConstants.kEncoderPositionFactor);
        m_shooterEncoderR.setVelocityConversionFactor(NotePipelineConstants.kEncoderVelocityFactor);
        

        m_shooterPIDR = m_shooterMotorR.getPIDController();
        m_shooterPIDR.setFeedbackDevice(m_shooterEncoderR);
        m_shooterPIDR.setP(0.1);
        m_shooterPIDR.setI(0.0);
        m_shooterPIDR.setD(0.0);
        m_shooterPIDR.setFF(0.03);
    }

    /**
     * @param mps set the motors to a speed in meters per second
     */
    public void set(double mps) {
        m_targetVelMPS = mps;
        System.out.println("Set to : " + m_targetVelMPS);
        m_shooterPIDL.setReference(-m_targetVelMPS, ControlType.kVelocity);
        m_shooterPIDR.setReference(m_targetVelMPS, ControlType.kVelocity);
    }

    @Override
    protected void dashboardPeriodic() {
        SmartDashboard.putNumber("Left Shooter Velocity", m_shooterEncoderL.getVelocity());
        SmartDashboard.putNumber("Right Shooter Velocity", m_shooterEncoderR.getVelocity());
        SmartDashboard.putNumber("Left Shooter Position", m_shooterEncoderL.getPosition());
        SmartDashboard.putNumber("Right Shooter Position", m_shooterEncoderR.getPosition());
        SmartDashboard.putNumber("Shooter Target Velocity", m_targetVelMPS);
    }
}
