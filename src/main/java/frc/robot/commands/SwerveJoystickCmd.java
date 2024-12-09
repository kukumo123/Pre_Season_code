package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve;

public class SwerveJoystickCmd extends Command {
    private final Swerve swerve;
    private final DoubleSupplier xSpdFunction, ySpdFunction, omegaFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(Swerve swerve,DoubleSupplier xSpdFunction, DoubleSupplier ySpdFunction, DoubleSupplier omegaFunction) {
        this.swerve = swerve;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.omegaFunction = omegaFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        double xSpeed, ySpeed, turningSpeed;

        xSpeed = xSpdFunction.getAsDouble();
        ySpeed = ySpdFunction.getAsDouble();
        turningSpeed = omegaFunction.getAsDouble();
        double linearMagnitude = MathUtil.applyDeadband(
                Math.hypot(xSpeed, ySpeed), OIConstants.kDeadband);
        Rotation2d linearDirection = new Rotation2d(xSpeed, ySpeed);
        double omega = MathUtil.applyDeadband(turningSpeed, OIConstants.kDeadband);

        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
        swerve.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xLimiter.calculate(linearVelocity.getX() * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond),
                        yLimiter.calculate(linearVelocity.getY() * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond),
                        turningLimiter.calculate(omega * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond),
                        swerve.getRotation2d()));
        }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

