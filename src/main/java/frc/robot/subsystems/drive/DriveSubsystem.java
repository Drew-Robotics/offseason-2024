package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.units.*;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final NetworkTable m_swerveTable;
  private final AHRS m_gyro;

  private boolean m_isFieldOriented = DriveConstants.kFieldOriented;

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final SwerveModule[] m_modules;

  private final SwerveDriveKinematics m_driveKinematics;

  private static DriveSubsystem m_instance;
  public static DriveSubsystem getInstance() {
    if (m_instance == null)
      m_instance = new DriveSubsystem();
    return m_instance;
  }

  protected DriveSubsystem() {
    m_swerveTable = NetworkTableInstance.getDefault().getTable("Swerve");
    m_gyro = new AHRS(SerialPort.Port.kMXP, AHRS.SerialDataType.kProcessedData, (byte) 50);

    m_frontLeft = new SwerveModule(
      "Front Left",
      DriveConstants.CanIDs.kFrontLeftDriving,
      DriveConstants.CanIDs.kFrontLeftTurning,
      DriveConstants.AngularOffsets.kFrontLeft,
      m_swerveTable
    );
    m_frontRight = new SwerveModule(
      "Front Right",
      DriveConstants.CanIDs.kFrontRightDriving,
      DriveConstants.CanIDs.kFrontRightTurning,
      DriveConstants.AngularOffsets.kFrontRight,
      m_swerveTable

    );
    m_backLeft = new SwerveModule(
      "Back Left",
      DriveConstants.CanIDs.kBackLeftDriving,
      DriveConstants.CanIDs.kBackLeftTurning,
      DriveConstants.AngularOffsets.kBackLeft,
      m_swerveTable
    );
    m_backRight = new SwerveModule(
      "Back Right",
      DriveConstants.CanIDs.kBackRightDriving,
      DriveConstants.CanIDs.kBackRightTurning,
      DriveConstants.AngularOffsets.kBackRight,
      m_swerveTable
    );

    m_modules = new SwerveModule[] {m_frontLeft, m_frontRight, m_backLeft, m_backRight};

    m_driveKinematics = new SwerveDriveKinematics(
      new Translation2d(DriveConstants.kWheelBase.divide(2), DriveConstants.kTrackWidth.divide(2)),
      new Translation2d(DriveConstants.kWheelBase.divide(2), DriveConstants.kTrackWidth.divide(2).negate()),
      new Translation2d(DriveConstants.kWheelBase.divide(2).negate(), DriveConstants.kTrackWidth.divide(2)),
      new Translation2d(DriveConstants.kWheelBase.divide(2).negate(), DriveConstants.kTrackWidth.divide(2).negate())
    );
  }

  @Override
  public void periodic() {

  }

  /**
   * Get the yaw from the Navx gyro.
   * 
   * @return Yaw of the Navx Gyro
   */
  public Rotation2d getGyroYaw() {
    return new Rotation2d(m_gyro.getYaw() * 2 * Math.PI * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
  }

  /**
   * Drive the robot.
   * 
   * @param x The x velocity, unitless on the range 0 to 1
   * @param y The y velocity, unitless on the range -1 to 1
   * @param rot The rotational velocity, unitless on the range -1 to 1
   */
  public void drive(double x, double y, double rot) {
    x = Math.max(-1, Math.min(1, x));
    y = Math.max(-1, Math.min(1, x));
    rot = Math.max(-1, Math.min(1, x));

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

  /**
   * Sets the states of the modules given an array of module states.
   * 
   * @param swerveModuleStates Array of modules states
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MaxVels.kTranslationalVelocity);
    for (int i = 0; i < 4; i++)
      m_modules[i].setModuleState(swerveModuleStates[i]);
  }
}
