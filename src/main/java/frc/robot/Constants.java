package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

public static final class DriveConstants {
    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 1.5 * 2 * Math.PI;
    public static final double kTrackWidth = Units.inchesToMeters(21);
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    public static final double kDriveBaseRadius = Math.hypot(kWheelBase / 2.0, kTrackWidth / 2.0);
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0),
        new Translation2d(DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0),
        new Translation2d(-DriveConstants.kWheelBase / 2.0, DriveConstants.kTrackWidth / 2.0),
        new Translation2d(-DriveConstants.kWheelBase / 2.0, -DriveConstants.kTrackWidth / 2.0)
};
public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.5;
public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
        kPhysicalMaxAngularSpeedRadiansPerSecond / 1;
public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 20;
public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 20;
}

public static final class ModuleConstants{
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4); //輪徑
    public static final double kDriveMotorGearRatio = 1 / 5.95;
    public static final double kTurningMotorGearRatio = 1 /21;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.3;
    public static final double kITurning = 0;
}

public static final class kFLModule{
    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kFrontLeftTurningMotorPort = 2;
    public static final Boolean kFrontLeftDriveReversed = false;
    public static final Boolean kFrontLeftTurningReversed = false;
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final Boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
}

public static final class kFRModule{
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final Boolean kFrontRightDriveReversed = false;
    public static final Boolean kFrontRightTurningReversed = false;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 2;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final Boolean kFrontRightDriveAbsoluteEncoderReversed = false;
}

public static final class kBLModule{
    public static final int kBackLeftDriveMotorPort = 5;
    public static final int kBackLeftTurningMotorPort = 6;
    public static final Boolean kBackLeftDriveReversed = false;
    public static final Boolean kBackLeftTurningReversed = false;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
    public static final Boolean kBackLeftDriveAbsoluteEncoderReversed = false;
}

public static final class kBRModule{
    public static final int kBackRightDriveMotorPort = 7;
    public static final int kBackRightTurningMotorPort = 8;
    public static final Boolean kBackRightDriveReversed = false;
    public static final Boolean kBackRightTurningReversed = false;
    public static final int kBackRightDriveAbsoluteEncoderPort = 4;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;
    public static final Boolean kBackRightDriveAbsoluteEncoderReversed = false;
}

public static final class OIConstants{
    public static final double kDeadband = 0.05;
    public static final int kDriverControllerPort = 0;
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;

}
}

