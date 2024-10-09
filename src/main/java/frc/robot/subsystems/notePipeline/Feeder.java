package frc.robot.subsystems.notePipeline;

import com.revrobotics.CANSparkBase.ControlType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.NotePipelineConstants.CANIDs;

public class Feeder {
    private final CANSparkFlex m_feederMotor;
    private final SparkPIDController m_feederPID;
    private final RelativeEncoder m_feederEncoder;

    public Feeder() {
        m_feederMotor = new CANSparkFlex(CANIDs.kFeeder, MotorType.kBrushless);
        m_feederEncoder = m_feederMotor.getEncoder();
        m_feederPID = m_feederMotor.getPIDController();
        m_feederPID.setFeedbackDevice(m_feederEncoder);
    }

    public void setMotor(double mps) {
        m_feederPID.setReference(mps, ControlType.kVelocity);
    }

    public DoubleSupplier getVelocity() {
        return () -> { return m_feederEncoder.getVelocity(); };
    }
}
