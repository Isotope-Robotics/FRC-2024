// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// e
package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Blinkin;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_AutonomousCommand;
  private RobotContainer auto_RobotContainer;

    //controller
   // private XboxController controller;
   // private CANSparkMax leftShooterMotor;
    //private CANSparkMax rightShooterMotor;
  

  // Swerve Varibles
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  public Swerve swerve;

  // Shooter Varibles
  private final Shooter shooter = Shooter.getInstance();

  // Intake Varibles
  private final Intake intake = Intake.getInstance();

  private final Blinkin blinkin = Blinkin.getInstance();

  public boolean bool = true;
  

  public void controls() {
  //  controller = new XboxController(0); // Change the port number as per your setup
   // leftShooterMotor = new CANSparkMax(1, MotorType.kBrushless); // Change the motor ID as per your setup
    //rightShooterMotor = new CANSparkMax(2, MotorType.kBrushless); // Change the motor ID as per your setup
}


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    swerve = Swerve.getInstance();
    //Robot Container for Auto Commands
    //auto_RobotContainer = new RobotContainer();

    // Zero Gyro Heading for Swerve
    //swerve.zeroHeading();

    // Zero Shooter and Intake Encoders
   // shooter.zeroEncoders();
   // intake.zeroEncoders();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //Command Scheduler ONLY for Auto
    //CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_AutonomousCommand = auto_RobotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_AutonomousCommand != null) {
      m_AutonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //Destory Auto Commands When Switching To TeleOP
    if (m_AutonomousCommand != null) {
      m_AutonomousCommand.cancel();
    }

    
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // By Default Swerve Is Field Relative
   
      SwerveDrive(false);

      // if (Constants.Controllers.driver2.getAButton()) {
      //   blinkin.scannerRed();
      // }
      // else if (Constants.Controllers.driver2.getXButton()) {
      //   blinkin.confetti();
      // }
      // else if (Constants.Controllers.driver2.getYButton()) {
      //   blinkin.rainbowRGB();
      // }
      // else if (Constants.Controllers.driver2.getBButton()) {
      //   blinkin.fireMedium();
      // }
      // else {
      //   blinkin.breathRed();
      // }

      // if (Constants.Controllers.driver2.getAButton()) {
      //   blinkin.index += 0.02;
      // }

      // if (Constants.Controllers.driver2.getBButton()) {
      //   blinkin.index -= 0.02;
      // }

      blinkin.index = Constants.Controllers.driver2.getRawAxis(0);

      blinkin.index();

      SmartDashboard.putBoolean("Note Intaked", intake.getNoteIntaked());
   
     //// Get the controller axis values
 //double shooterSpeed = controller.getRawAxis(3); // Change the axis number as per your setup

 // Map the controller output to motor speeds
// double leftMotorSpeed = -shooterSpeed; // Reverse the value if necessary
// double rightMotorSpeed = shooterSpeed;

 // Set the motor speeds :D
 //leftShooterMotor.set(leftMotorSpeed);
 //rightShooterMotor.set(rightMotorSpeed);

    // Zero Heading if Left Bumper is Pushed
   // if (Constants.Controllers.driver1.getLeftBumperPressed()) {
    //  swerve.zeroHeading();
   // }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  // To Drive With Controllers
  private void SwerveDrive(boolean isFieldRel) {
    // Controller Deadbands (Translation, Strafe, Rotation)
    final double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1),
        Constants.Controllers.stickDeadband);
    final double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0),
        Constants.Controllers.stickDeadband);
    final double rot = MathUtil.applyDeadband(-Constants.Controllers.driver1.getRawAxis(2),
        Constants.Controllers.stickDeadband);

    // Drive Function
    swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);
  }

  // Adding Auto Commands to Auto Selector
  public void generateAutos() {
    SmartDashboard.putData("Example Auto", new PathPlannerAuto("ExampleAuto"));
  }
}
