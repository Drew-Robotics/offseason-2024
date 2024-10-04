package frc.robot.subsystems.notePipeline;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.NotePipeLineConstants.CANIDs;

public class Intake {
    private final CANSparkFlex m_intakeMotor;
    private final SparkPIDController m_intakePID;
    private final RelativeEncoder m_intakeEncoder;

    public Intake() {
        m_intakeMotor = new CANSparkFlex(CANIDs.kIntake, MotorType.kBrushless);
        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_intakePID = m_intakeMotor.getPIDController();
        m_intakePID.setFeedbackDevice(m_intakeEncoder);
    }

    public void setMotor(double mps) {
        m_intakePID.setReference(mps, ControlType.kVelocity);
    }
}
