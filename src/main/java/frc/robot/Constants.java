package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class Constants {

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 8.31;
    public static final double kTurningMotorGearRatio = 1 / 18.0;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
  }

  public static final class DriveConstants {
    // distances are set

    public static final double kTrackWidth = Units.inchesToMeters(23);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(23);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    // CAN ports are set

    public static final int kFrontLeftDriveMotorPort = 35;
    public static final int kBackLeftDriveMotorPort = 17;
    public static final int kFrontRightDriveMotorPort = 23;
    public static final int kBackRightDriveMotorPort = 10;

    public static final int kFrontLeftTurningMotorPort = 15;
    public static final int kBackLeftTurningMotorPort = 18;
    public static final int kFrontRightTurningMotorPort = 29;
    public static final int kBackRightTurningMotorPort = 21;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 1;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
    public static final int kBackRightDriveAbsoluteEncoderPort = 2;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 5.19;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.07;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 6.27;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 3.41;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
        / 2;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    public static final double balanceP = 0.1;
    public static final double balanceI = 0;
    public static final double balanceD = 0;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = //
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kPXController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPThetaController = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverTurnControllerPort = 1;
    public static final int kPXNControllerPort = 2;

    public static final class pxnButtons {
      public static final int L1 = 5;
      // public static final int L2 = 7; don't use this
      public static final int R1 = 6;
      // public static final int R2 = 8; don't use this
      public static final int A = 2;
      public static final int B = 3;
      public static final int X = 1;
      public static final int Y = 4;
      public static final int Share = 9;
      public static final int Options = 10;
      public static final int L3 = 11;
      public static final int R3 = 12;
    }

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.05;
  }

  public static class ColorConstants {
    public static final Color8Bit BLACK = new Color8Bit(0, 0, 0);
    public static final Color8Bit RED = new Color8Bit(204, 0, 0);
    public static final Color8Bit ORANGE = new Color8Bit(204, 84, 0);
    public static final Color8Bit YELLOW = new Color8Bit(204, 204, 0);
    public static final Color8Bit GREEN = new Color8Bit(0, 204, 0);
    public static final Color8Bit BLUE = new Color8Bit(0, 0, 204);
    public static final Color8Bit PURPLE = new Color8Bit(152, 16, 201);
    public static final Color8Bit COLORS[] = { RED, ORANGE, YELLOW, GREEN, BLUE, PURPLE };
  }
}