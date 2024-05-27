package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;

public class Constants {
  public static final class DriveConstants {

    public static final Measure<Distance> kWheelBase = Units.Meters.of(0);
    public static final Measure<Distance> kTrackWidth = Units.Meters.of(0);

    public static final Boolean kGyroReversed = false;
    
    public static final class CanIDs {
      public static final int kFrontLeftDriving = 3;
      public static final int kBackLeftDriving = 1;
      public static final int kFrontRightDriving = 7;
      public static final int kBackRightDriving = 5;

      public static final int kFrontLeftTurning = 4;
      public static final int kBackLeftTurning = 2;
      public static final int kFrontRightTurning = 8;
      public static final int kBackRightTurning = 6;
    }
    public static final class AngularOffsets {
      public static final Rotation2d kFrontLeft = new Rotation2d(-Math.PI / 2);
      public static final Rotation2d kFrontRight = new Rotation2d(0);
      public static final Rotation2d kBackLeft = new Rotation2d(Math.PI);
      public static final Rotation2d kBackRight = new Rotation2d(Math.PI / 2);
    }
    public static final class MaxVels {
      public static final Measure<Velocity<Distance>> kTranslationalVelocity = Units.MetersPerSecond.of(1);
      public static final Measure<Velocity<Angle>> kRotationalVelocity = Units.RadiansPerSecond.of(1);
    }
  }

  public static final class SwerveCalculations {
    public static final double kNeoFreeSpeedRpm = 5676d;
    public static final double kDrivingMotorFreeSpeedRps = kNeoFreeSpeedRpm / 60d;

    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    
    public static final int kDrivingMotorPinionTeeth = 14;
    public static final int kSpurGearTeeth = 20;

    // 45 teeth on the wheels bevel gear and 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (double) (45d * kSpurGearTeeth) / (double) (kDrivingMotorPinionTeeth * 15d);

    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
    // convert from rotations to meters
    public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters / kDrivingMotorReduction;
  }

  public static final class SwerveConstants {

    public static final double kDrivingP = 0;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / SwerveCalculations.kDriveWheelFreeSpeedRps;

    public static final double kTurningP = 0;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;

    public static final int kDrivingMotorCurrentLimit = 50;
    public static final int kTurningMotorCurrentLimit = 20;

    // go from rotations or rotations per minute to meters or meters per second
    public static final double kDrivingEncoderPositionFactor = SwerveCalculations.kDrivingEncoderPositionFactor;
  }
}
