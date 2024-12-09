package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.Swerve;


public class RobotContainer {


  
    private final Swerve swerveSubsystem = new Swerve();
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private static final String kDefaultAuto = "New Auto";
    private static final String middleStart = "3 Note Auto";
    private static final String RightStart = "Auto2";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        m_chooser.setDefaultOption("New Auto", kDefaultAuto);
        m_chooser.addOption("3 Note Auto", middleStart);
        m_chooser.addOption("Auto2", RightStart);
        SmartDashboard.putData("Auto choices", m_chooser);  
        m_autoSelected =  m_chooser.getSelected();
        System.out.println("Auto Selected" + m_autoSelected);

        switch (m_autoSelected) {
          case middleStart:
            break;
          case RightStart:
            break;
          case kDefaultAuto:
          default:
          break;
        }

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis)));
        configureButtonBindings();     
    }

  
    private void configureButtonBindings() {
    }


      public PathPlannerAuto getAutonomousCommand(){
        return new PathPlannerAuto(m_chooser.getSelected());
      }
  }