package frc.robot.subsystems.notePipeline;

import com.revrobotics.CANSparkBase.ControlType;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.NotePipelineConstants;
import frc.robot.Constants.NotePipelineConstants.CANIDs;
import frc.robot.Constants.NotePipelineConstants.PID;
import frc.robot.Constants.NotePipelineConstants.Sensor;

public class FeederSubsystem extends PipelineSubsystem {
    private final CANSparkFlex m_feederMotor;
    private final SparkPIDController m_feederPID;
    private final RelativeEncoder m_feederEncoder;

    private final TimeOfFlight m_sensor = new TimeOfFlight(CANIDs.kSensor);

    private static FeederSubsystem m_instance;

    private boolean m_intook; // scientific term for the past tense of intake totally

    public static FeederSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new FeederSubsystem();
        return m_instance;
    }

    public FeederSubsystem() {
        super(FeederSubsystem.class.getSimpleName());
        m_feederMotor = new CANSparkFlex(CANIDs.kFeeder, MotorType.kBrushless);
        m_feederMotor.setInverted(true);

        m_feederEncoder = m_feederMotor.getEncoder();
        m_feederEncoder.setPositionConversionFactor(NotePipelineConstants.kFeederPositionFactor);
        m_feederEncoder.setVelocityConversionFactor(NotePipelineConstants.kFeederVelocityFactor);

        m_feederPID = m_feederMotor.getPIDController();
        m_feederPID.setFeedbackDevice(m_feederEncoder);
        m_feederPID.setP(PID.Feeder.kP);
        m_feederPID.setI(PID.Feeder.kI);
        m_feederPID.setD(PID.Feeder.kD);
        m_feederPID.setFF(PID.Feeder.kFF);

        m_sensor.setRangingMode(RangingMode.Short, Sensor.checkTime);
    }

    public void set(double mps) {
        // SmartDashboard.putNumber("Feeder Target Velocity", mps);
        m_feederPID.setReference(mps, ControlType.kVelocity);
    }

    public boolean hasNote() {
        return m_sensor.getRange() <= Sensor.noteRange;
    }

    public boolean hasIntook() { return m_intook; }
    public void intook(boolean intook) { m_intook = intook; }

    @Override
    protected void dashboardPeriodic() {
        SmartDashboard.putNumber("Feeder Velocity", m_feederEncoder.getVelocity());
        SmartDashboard.putNumber("Feeder Position", m_feederEncoder.getPosition());
        SmartDashboard.putNumber("Sensor Distance", m_sensor.getRange());
        SmartDashboard.putBoolean("Intook", m_intook);
    }
}
