package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.kBRModule;
import frc.robot.Constants.kFLModule;
import frc.robot.Constants.kFRModule;
import frc.robot.Constants.kBLModule;


public class Swerve extends SubsystemBase {
    private final AHRS gyro;
    private final SwerveModule[] modules = new SwerveModule[4];
    StructArrayPublisher<SwerveModuleState> publisherSwerveModuleState = NetworkTableInstance.getDefault()
            .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
    StructPublisher<Pose3d> publisher3D = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose3d.struct).publish();
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.kModuleTranslations);
    private SwerveModulePosition[] lastModulePositions;// For delta tracking
    private Pose2d m_pose;
    private SwerveModulePosition[] modulePositions;
    private SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    private Field2d field = new Field2d();
    private SwerveDriveOdometry m_Odometry;
    private SwerveDriveKinematics m_Kinematics;
    Pose2d poseA = new Pose2d();
    Pose2d poseB = new Pose2d();        

        
    public Swerve() {
            gyro = new AHRS(SPI.Port.kMXP);{
            new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();}

            Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
     m_Kinematics = new SwerveDriveKinematics(
                m_backLeftLocation, m_backRightLocation, m_frontLeftLocation, m_frontRightLocation);
        modules[0] = new SwerveModule(
                kFLModule.kFrontLeftDriveMotorPort,
                kFLModule.kFrontLeftTurningMotorPort,
                kFLModule.kFrontLeftDriveReversed,
                kFLModule.kFrontLeftTurningReversed,
                kFLModule.kFrontLeftDriveAbsoluteEncoderPort,
                kFLModule.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                kFLModule.kFrontLeftDriveAbsoluteEncoderReversed);

        modules[1] = new SwerveModule(
                kFRModule.kFrontRightDriveMotorPort,
                kFRModule.kFrontRightTurningMotorPort,
                kFRModule.kFrontRightDriveReversed,
                kFRModule.kFrontRightTurningReversed,
                kFRModule.kFrontRightDriveAbsoluteEncoderPort,
                kFRModule.kFrontRightDriveAbsoluteEncoderOffsetRad,
                kFRModule.kFrontRightDriveAbsoluteEncoderReversed);

        modules[2] = new SwerveModule(
                kBLModule.kBackLeftDriveMotorPort,
                kBLModule.kBackLeftTurningMotorPort,
                kBLModule.kBackLeftDriveReversed,
                kBLModule.kBackLeftTurningReversed,
                kBLModule.kBackLeftDriveAbsoluteEncoderPort,
                kBLModule.kBackLeftDriveAbsoluteEncoderOffsetRad,
                kBLModule.kBackLeftDriveAbsoluteEncoderReversed);

        modules[3] = new SwerveModule(
                kBRModule.kBackRightDriveMotorPort,
                kBRModule.kBackRightTurningMotorPort,
                kBRModule.kBackRightDriveReversed,
                kBRModule.kBackRightTurningReversed,
                kBRModule.kBackRightDriveAbsoluteEncoderPort,
                kBRModule.kBackRightDriveAbsoluteEncoderOffsetRad,
                kBRModule.kBackRightDriveAbsoluteEncoderReversed);
        lastModulePositions = // For delta tracking
                new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()
                };
        modulePositions = getModulePositions();

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose, 
                this::getSpeeds, 
                this::runVelocity, 
                new HolonomicPathFollowerConfig( 
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0),
                        DriveConstants.kPhysicalMaxSpeedMetersPerSecond, 
                        DriveConstants.kDriveBaseRadius, 
                        new ReplanningConfig()
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", field);

        m_Odometry = new SwerveDriveOdometry(
        m_Kinematics, gyro.getRotation2d(),
            new SwerveModulePosition[] {
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition()
            }, new Pose2d(5.0, 13.5, new Rotation2d()));
    }

    public void zeroHeading() {
        if (gyro.isConnected()) {
            gyro.reset();
        }
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(this.getSweveModuleStates());
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    public SwerveModuleState[] getSweveModuleStates(){
        return new SwerveModuleState[] {
        modules[0].getState(),
        modules[1].getState(),
        modules[2].getState(),
        modules[3].getState()
        };
        }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return m_Odometry.getPoseMeters();
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] position = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            position[i] = modules[i].getPosition();
        }
        return position;
    }

    public void runVelocity(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveModuleState[] optimizedSetpointStates = setModuleStates(setpointStates);
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        var swerveModuleStates = m_Kinematics.toSwerveModuleStates(chassisSpeeds);
        modules[0].setDesiredState(swerveModuleStates[0]);
        modules[1].setDesiredState(swerveModuleStates[1]);
        modules[2].setDesiredState(swerveModuleStates[2]);
        modules[3].setDesiredState(swerveModuleStates[3]);
        setModuleStates(swerveModuleStates);
        setStates(targetStates);
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized",
                optimizedSetpointStates);
    }

    public void setStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    
        for (int i = 0; i < modules.length; i++) {
          modules[i].setTargetState(targetStates[i]);
        }
      }

    public SwerveModuleState[] setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            optimizedSetpointStates[i] = modules[i].setDesiredState(desiredStates[i]);
        }
        return optimizedSetpointStates;
    }

    public void resetPose(Pose2d pose) {
        m_Odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            modulePositions[moduleIndex] = modules[moduleIndex].getPosition();
            moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters
                            - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            SmartDashboard.putString("AbsoluteEncoder[" + moduleIndex + "]",
                    String.valueOf(modules[moduleIndex].getAbsoluteEncoderRad()));
            SmartDashboard.putString("RelativeEncoder[" + moduleIndex + "]",
                    String.valueOf(modules[moduleIndex].getTurningPosition()));
        }

        publisherSwerveModuleState.set(getSweveModuleStates());
        field.setRobotPose(getPose());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        var gyroAngle = gyro.getRotation2d();

        m_pose = m_Odometry.update(gyroAngle, new SwerveModulePosition[]{
            modules[0].getPosition(),modules[1].getPosition(),
            modules[2].getPosition(),modules[3].getPosition()
        });

        publisher.set(m_pose);
        arrayPublisher.set(new Pose2d[] {m_pose});
        Logger.recordOutput("MyPose", m_pose);

    }

    public void stopModules() {
        for (int i = 0; i < 4; i++) {
            modules[i].stop();
        }
    }

    public void resetGyro() {
        gyro.reset();
    }
}
