package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private static DriveSubsystem m_instance;
  NetworkTable m_swerveTable = NetworkTableInstance.getDefault().getTable("Swerve");

  private final AHRS m_gyro = new AHRS(SerialPort.Port.kMXP, AHRS.SerialDataType.kProcessedData, (byte) 50);

  private boolean m_isFieldOriented = false;

  private final SwerveModule m_frontLeft = new SwerveModule(
    "Front Left",
    DriveConstants.CanIDs.kFrontLeftDriving,
    DriveConstants.CanIDs.kFrontLeftTurning,
    DriveConstants.AngularOffsets.kFrontLeft,
    m_swerveTable
  );
  private final SwerveModule m_frontRight = new SwerveModule(
    "Front Right",
    DriveConstants.CanIDs.kFrontRightDriving,
    DriveConstants.CanIDs.kFrontRightTurning,
    DriveConstants.AngularOffsets.kFrontRight,
    m_swerveTable

  );
  private final SwerveModule m_backLeft = new SwerveModule(
    "Back Left",
    DriveConstants.CanIDs.kBackLeftDriving,
    DriveConstants.CanIDs.kBackLeftTurning,
    DriveConstants.AngularOffsets.kBackLeft,
    m_swerveTable
  );
  private final SwerveModule m_backRight = new SwerveModule(
    "Back Right",
    DriveConstants.CanIDs.kBackRightDriving,
    DriveConstants.CanIDs.kBackRightTurning,
    DriveConstants.AngularOffsets.kBackRight,
    m_swerveTable
  );

  private final SwerveModule[] modules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};

  private final SwerveDriveKinematics m_driveKinematics = new SwerveDriveKinematics(
    new Translation2d(DriveConstants.kWheelBase.divide(2), DriveConstants.kTrackWidth.divide(2)),
    new Translation2d(DriveConstants.kWheelBase.divide(2), DriveConstants.kTrackWidth.divide(2).negate()),
    new Translation2d(DriveConstants.kWheelBase.divide(2).negate(), DriveConstants.kTrackWidth.divide(2)),
    new Translation2d(DriveConstants.kWheelBase.divide(2).negate(), DriveConstants.kTrackWidth.divide(2).negate())
  );

  public static DriveSubsystem getInstance() {
    if (m_instance == null)
      m_instance = new DriveSubsystem();
    return m_instance;
  }

  protected DriveSubsystem() {

  }

  @Override
  public void periodic() {

  }

  public Rotation2d getGyroYaw() {
    return new Rotation2d(m_gyro.getYaw() * 2 * Math.PI * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
  }

  public void drive(double x, double y, double rot) {
    Measure<Velocity<Distance>> xVel = DriveConstants.MaxVels.kTranslationalVelocity.times(x);
    Measure<Velocity<Distance>> yVel = DriveConstants.MaxVels.kTranslationalVelocity.times(y);
    Measure<Velocity<Angle>> rotVel = DriveConstants.MaxVels.kRotationalVelocity.times(rot);

    ChassisSpeeds commandedChassisSpeeds;
    if (m_isFieldOriented)
      commandedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, getGyroYaw());
    else
      commandedChassisSpeeds = new ChassisSpeeds(xVel, yVel, rotVel);
      
    SwerveModuleState[] swerveModuleStates = m_driveKinematics.toSwerveModuleStates(commandedChassisSpeeds);
    setModuleStates(swerveModuleStates);
  }
  
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MaxVels.kTranslationalVelocity);
    for (int i = 0; i < 4; i++)
      modules[i].setModuleState(swerveModuleStates[i]);
  }
}
