package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.units.*;
import edu.wpi.first.util.WPIUtilJNI;
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

  private SlewRateLimiter m_magnitudeLimiter;
  private SlewRateLimiter m_directionalLimiter;
  private SlewRateLimiter m_rotationLimiter;

  private Measure<Velocity<Distance>> m_xVelocity = Units.MetersPerSecond.of(0);
  private Measure<Velocity<Distance>> m_yVelocity = Units.MetersPerSecond.of(0);
  private Measure<Velocity<Angle>> m_rotationalVelocity = Units.RadiansPerSecond.of(0);

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

    m_magnitudeLimiter = new SlewRateLimiter(DriveConstants.SlewRate.kMag);
    m_directionalLimiter = new SlewRateLimiter(DriveConstants.SlewRate.kDir);
    m_rotationLimiter = new SlewRateLimiter(DriveConstants.SlewRate.kRot);
  }

  @Override
  public void periodic() {

  }

  private void rateLimit() {

    // double translationVelMagnitude = 
    //   Math.sqrt(Math.pow(m_xVelocity.in(Units.MetersPerSecond), 2) + Math.pow(m_yVelocity.in(Units.MetersPerSecond), 2));

    // double translationVelDirection = 
    //   Math.atan2(m_xVelocity.in(Units.MetersPerSecond), m_yVelocity.in(Units.MetersPerSecond)); // arctan(y / x)

    // double rotationalVel = m_rotationalVelocity.in(Units.RadiansPerSecond);

    // double lastTranslationVelMagnitude = m_translationVelMagnitude.in(Units.MetersPerSecond);
    // double lastTranslationVelDirection = m_translationVelDirection.in(Units.RadiansPerSecond);
    // double lastRotationalVel = m_rotationalVelocity.in(Units.RadiansPerSecond);

    // double newTranslationVelMagnitude;
    // double newTranslationVelDirection;
    // double newRotationalVel;

    //   // Calculate the direction slew rate based on an estimate of the lateral acceleration
    //   double directionSlewRate;
    //   if (lastTranslationVelMagnitude != 0.0) {
    //     directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_translationVelMagnitude);
    //   } else {
    //     directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
    //   }

    //   double currentTime = WPIUtilJNI.now() * 1e-6;
    //   double elapsedTime = currentTime - m_prevTime;
      
    //   double angularAccel = Math.abs(translationVelDirection - lastTranslationVelMagnitude); // acceleration estimate
    //   angularAccel = angularAccel > Math.PI ? (2 * Math.PI) - angularAccel : angularAccel;

    //   if (angularAccel < 0.45 * Math.PI) { // if the angular accel is small enough
    //     newTranslationVelMagnitude = m_magnitudeLimiter.calculate(lastTranslationVelMagnitude);
    //     newTranslationVelDirection = SwerveUtils.StepTowardsCircular(lastTranslationVelDirection, translationVelDirection, directionSlewRate * elapsedTime);
    //   }
    //   else if (angularAccel > 0.85 * Math.PI && lastTranslationVelMagnitude > 1e-4) {
    //     newTranslationVelMagnitude = m_magnitudeLimiter.calculate(0.0);
    //     newTranslationVelDirection = lastTranslationVelDirection;
    //   }
    //   else if (angularAccel > 0.85 * Math.PI) {
    //     newTranslationVelMagnitude = m_magnitudeLimiter.calculate(translationVelMagnitude);
    //     newTranslationVelDirection = SwerveUtils.WrapAngle(lastTranslationVelDirection + Math.PI);
    //   }
    //   else {
    //     newTranslationVelMagnitude = m_magnitudeLimiter.calculate(0.0);
    //     newTranslationVelDirection = SwerveUtils.StepTowardsCircular(lastTranslationVelDirection, translationVelDirection, directionSlewRate * elapsedTime);
    //   }
    //   m_prevTime = currentTime;
      
    //   m_translationVelMagnitude = Units.MetersPerSecond.of(newTranslationVelMagnitude);
    //   m_translationVelDirection = Units.RadiansPerSecond.of(newTranslationVelDirection);

    //   m_xVelocity = Units.MetersPerSecond.of(lastTranslationVelMagnitude * Math.cos(newTranslationVelDirection));
    //   m_yVelocity = Units.MetersPerSecond.of(lastTranslationVelMagnitude * Math.sin(newTranslationVelDirection));
    //   m_rotationalVelocity = Units.RadiansPerSecond.of(m_rotationLimiter.calculate(rotationalVel));
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

    m_xVelocity = DriveConstants.MaxVels.kTranslationalVelocity.times(x);
    m_yVelocity = DriveConstants.MaxVels.kTranslationalVelocity.times(y);
    m_rotationalVelocity = DriveConstants.MaxVels.kRotationalVelocity.times(rot);

    // convert to polar

    double xv, yv, rv, mv, dv;
  
    xv = m_xVelocity.in(Units.MetersPerSecond);
    yv = m_yVelocity.in(Units.MetersPerSecond);
    rv = m_rotationalVelocity.in(Units.RadiansPerSecond);

    mv = Math.sqrt(Math.pow(xv, 2) + Math.pow(yv, 2));
    dv = Math.atan2(yv, xv);

    // rate limit

    mv = m_magnitudeLimiter.calculate(mv);
    dv = m_directionalLimiter.calculate(dv);
    rv = m_rotationLimiter.calculate(rv);

    // convert back and store units

    xv = mv * Math.cos(dv);
    yv = mv * Math.sin(dv);
    
    m_xVelocity = Units.MetersPerSecond.of(xv);
    m_yVelocity = Units.MetersPerSecond.of(xv);
    m_rotationalVelocity = Units.RadiansPerSecond.of(xv);

    // get chassis speeds

    ChassisSpeeds commandedChassisSpeeds;
    if (m_isFieldOriented)
      commandedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_xVelocity, m_yVelocity, m_rotationalVelocity, getGyroYaw());
    else
      commandedChassisSpeeds = new ChassisSpeeds(m_xVelocity, m_yVelocity, m_rotationalVelocity);
      
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
