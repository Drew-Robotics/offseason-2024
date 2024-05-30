package frc.robot.subsystems.drive;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;

import edu.wpi.first.units.*;

import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

  private SwerveModuleState m_measuredState = new SwerveModuleState();
  private SwerveModuleState m_commandedState = new SwerveModuleState();
  private SwerveModulePosition m_measuredPosition = new SwerveModulePosition();

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPID;
  private final SparkPIDController m_turningPID;

  private final String m_moduleName;
  private final Rotation2d m_angularOffset;

  private final NetworkTable m_moduleTable;

  private final StructTopic<SwerveModuleState> m_measuredStateTopic;
  private final StructPublisher<SwerveModuleState> m_measuredStatePublisher;

  private final StructTopic<SwerveModuleState> m_commandedStateTopic;
  private final StructPublisher<SwerveModuleState> m_commandedStatePublisher;

  private final StructTopic<SwerveModulePosition> m_measuredPositionTopic;
  private final StructPublisher<SwerveModulePosition> m_measuredPositionPublisher;

  /**
   * Swerve Module Constructor.
   * 
   * @param driveMotorCanID Can ID of the driving Motor
   * @param turningMotorCanID Can ID of the turning motor
   * @param moduleName The name of the module. Ex: Front Right, Back Left 
   * @param angularOffset The angle offset of the module
   * @param superTable The network table for the module to publish its table under
   */
  public SwerveModule(String moduleName, int driveMotorCanID, int turningMotorCanID, Rotation2d angularOffset, NetworkTable superTable) {
    m_moduleName = moduleName;

    m_angularOffset = angularOffset;

    m_moduleTable = superTable.getSubTable(m_moduleName);

    m_measuredStateTopic = m_moduleTable.getStructTopic(moduleName + " Measured State", SwerveModuleState.struct);
    m_measuredStatePublisher = m_measuredStateTopic.publish();
    m_commandedStateTopic = m_moduleTable.getStructTopic(moduleName + " Commanded State", SwerveModuleState.struct);
    m_commandedStatePublisher = m_commandedStateTopic.publish();
    m_measuredPositionTopic = m_moduleTable.getStructTopic(moduleName + " Measured Position", SwerveModulePosition.struct);
    m_measuredPositionPublisher = m_measuredPositionTopic.publish();

    m_driveMotor = new CANSparkMax(driveMotorCanID, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorCanID, MotorType.kBrushless);

    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();

    m_drivingEncoder = m_driveMotor.getEncoder();
    m_drivingPID = m_driveMotor.getPIDController();
    m_drivingPID.setFeedbackDevice(m_drivingEncoder);
    m_driveMotor.setIdleMode(IdleMode.kBrake);

    // turning motor uses absolute encoder
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_turningPID = m_turningMotor.getPIDController();
    m_turningPID.setFeedbackDevice(m_turningEncoder);
    m_turningMotor.setIdleMode(IdleMode.kBrake);

    // go from rotations or rotations per minute to meters or meters per second
    m_drivingEncoder.setPositionConversionFactor(SwerveConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(SwerveConstants.kDrivingEncoderPositionFactor / 60);

    // go from rotations or rotations per minute to meters or meters per second
    m_turningEncoder.setPositionConversionFactor(2 * Math.PI);
    m_turningEncoder.setVelocityConversionFactor(2 * Math.PI / 60);

    // turning motor must be inverted, output shaft rotates in the opposite direction of turning
    m_turningEncoder.setInverted(true);

    // set up pid controllers
    m_drivingPID.setP(SwerveConstants.DrivingPID.kP);
    m_drivingPID.setI(SwerveConstants.DrivingPID.kI);
    m_drivingPID.setD(SwerveConstants.DrivingPID.kD);
    m_drivingPID.setFF(SwerveConstants.DrivingPID.kFF);
    m_drivingPID.setOutputRange(-1, 1);

    m_turningPID.setP(SwerveConstants.TurningPID.kP);
    m_turningPID.setI(SwerveConstants.TurningPID.kI);
    m_turningPID.setD(SwerveConstants.TurningPID.kD);
    m_turningPID.setFF(SwerveConstants.TurningPID.kFF);
    m_turningPID.setOutputRange(-1, 1);

    // current limits
    m_driveMotor.setSmartCurrentLimit((int) SwerveConstants.kDrivingMotorCurrentLimit.in(Units.Volts));
    m_turningMotor.setSmartCurrentLimit((int) SwerveConstants.kTurningMotorCurrentLimit.in(Units.Volts));

    m_driveMotor.burnFlash();
    m_turningMotor.burnFlash();
  }

  /**
   * Get the the state of the module. Encodes driving velocity and turning angle.
   * 
   * @return Module State
   */
  public SwerveModuleState getModuleState() {
    SwerveModuleState moduleState = new SwerveModuleState(
      Units.MetersPerSecond.of(m_drivingEncoder.getVelocity()),
      new Rotation2d(m_turningEncoder.getPosition()).minus(m_angularOffset)
    );

    m_measuredState = moduleState;

    m_measuredStatePublisher.set(m_measuredState);
    return m_measuredState;
  }

  /**
   * Command a module state. Param encodes driving velocity and turning angle.
   * 
   * @param moduleState Module State
   */
  public void setModuleState(SwerveModuleState moduleState) {
    SwerveModuleState correctedState = new SwerveModuleState(
      moduleState.speedMetersPerSecond,
      moduleState.angle.plus(m_angularOffset)
    );

    // avoid spining more than 90 degrees
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedState, new Rotation2d(m_turningEncoder.getPosition()));

    m_drivingPID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_commandedState = moduleState;

    m_commandedStatePublisher.set(m_commandedState);
  }

  /**
   * Gets the modules position object. Encodes driving distance and turning angle.
   * 
   * @return Module Position
   */
  public SwerveModulePosition getModulePosition() {
    SwerveModulePosition modulePosition = new SwerveModulePosition(
        Units.Meters.of(m_drivingEncoder.getPosition()),
        new Rotation2d(m_turningEncoder.getPosition()).minus(m_angularOffset)
    );

    m_measuredPosition = modulePosition;

    m_measuredPositionPublisher.set(m_measuredPosition);
    return m_measuredPosition;
  }
}
