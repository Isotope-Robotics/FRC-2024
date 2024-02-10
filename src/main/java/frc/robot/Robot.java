// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// e
package frc.robot;

<<<<<<< Updated upstream
import edu.wpi.first.math.MathUtil;
=======

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.MathUtil;
import java.math.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
>>>>>>> Stashed changes
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Climber;

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
  private RobotContainer robotContainer;

  // controller
  // private XboxController controller;
  // private CANSparkMax leftShooterMotor;
  // private CANSparkMax rightShooterMotor;

  // Swerve Varibles
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  public Swerve swerve = Swerve.getInstance();

  // Shooter Varibles
  private final Shooter shooter = Shooter.getInstance();

  // Intake Varibles
  private final Intake intake = Intake.getInstance();

  // private final Blinkin blinkin = Blinkin.getInstance();

  private final Climber climber = Climber.getInstance();

  private final Vision photonCannon = Vision.getInstance();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Robot Container for Auto Commands
    //robotContainer = new RobotContainer();

    // Zero Gyro Heading for Swerve
    swerve.zeroHeading();
    swerve.swerveOdometry.resetPosition(swerve.getGyroYaw(), swerve.getModulePositions(), swerve.getPose());

    // Zero Shooter and Intake Encoders
    // shooter.zeroEncoders();
    intake.zeroEncoders();
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
    // Command Scheduler ONLY for Auto
    CommandScheduler.getInstance().run();
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

  //  m_AutonomousCommand = robotContainer.getAutonomousCommand();

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
    // Destory Auto Commands When Switching To TeleOP
    if (m_AutonomousCommand != null) {
      m_AutonomousCommand.cancel();
    }

    intake.zeroEncoders();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   // System.out.println(intake.wristEncoder1.getPosition());

    // By Default Swerve Is Field Relative

    

<<<<<<< Updated upstream
    // Auto Aim to Target
    if (Constants.Controllers.driver1.getRawButton(10)) {
      SwerveAutoAim(true);
    }
=======
    //Auto Aim to Target
    
>>>>>>> Stashed changes

    // if (Constants.Controllers.driver2.getAButton()) {
    // blinkin.scannerRed();
    // }
    // else if (Constants.Controllers.driver2.getXButton()) {
    // blinkin.confetti();
    // }
    // else if (Constants.Controllers.driver2.getYButton()) {
    // blinkin.rainbowRGB();
    // }
    // else if (Constants.Controllers.driver2.getBButton()) {
    // blinkin.fireMedium();
    // }
    // else {
    // blinkin.breathRed();
    // }

    // if (Constants.Controllers.driver2.getAButton()) {
    // blinkin.index += 0.02;
    // }

    // if (Constants.Controllers.driver2.getBButton()) {
    // blinkin.index -= 0.02;
    // }

    SmartDashboard.putBoolean("Note Intaked", intake.getNoteIntaked());
    SmartDashboard.putBoolean("magnet", climber.getmagnet());

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    // Intake down & Start Intake
    if (Constants.Controllers.driver2.getLeftBumper()) {
<<<<<<< Updated upstream
      // intake.wristDown();
      intake.intakeStart(-.5);
    } else if (Constants.Controllers.driver2.getRightBumper()) {
      intake.intakeStart(.5);

    } else if (Constants.Controllers.driver2.getLeftTriggerAxis() > 0.5) {
      intake.intakeStart(-1);

    }

    else {
=======
     // intake.wristDown();
      intake.intakeStart(-0.5);
    } else if (Constants.Controllers.driver2.getRightBumper() && intake.getNoteIntaked()) {
      intake.intakeStart(0.5);
    } else {
>>>>>>> Stashed changes
      intake.intakeStop();
    }

    // Back to robot centric while button seven is pushed
    if (Constants.Controllers.driver1.getRawButton(2)) {
      swerve.zeroHeading();
      System.out.println("RESET GYRO!!!!!!!");
    }

    if (Constants.Controllers.driver1.getRawButton((1))) {
      SwerveDrive(false);
      System.out.println("HROBOT CENTRIC ENABLLEEEDDDDD!!");
    } else if (Constants.Controllers.driver1.getRawButton(4)){
      SwerveAutoAim(true);
    } else {
    SwerveDrive(true);

    }

<<<<<<< Updated upstream
    if (Constants.Controllers.driver2.getYButton()) {
      shooter.shoot(1.0);
=======


    

    if (Constants.Controllers.driver2.getYButton() && shooter.getNoteDetected()) {
      shooter.shoot(.5);
>>>>>>> Stashed changes
    } else if (Constants.Controllers.driver2.getXButton()) {
      shooter.shoot(-1.0);
    } else {
      shooter.stop();
    }

    if (Constants.Controllers.driver2.getBButton()) {
      intake.wristDown();
    } else if (Constants.Controllers.driver2.getBackButton()) {
      intake.wristHalf();
    } else {
      intake.wristUp();
    }

    if (intake.getWristLimit()) {
      intake.zeroEncoders();
    }
    

    if (Constants.Controllers.driver2.getPOV() == 0) {
      climber.extend();
    }

    if (Constants.Controllers.driver2.getPOV() == 180) {
      climber.retract();
    }

    System.out.println(intake.wristEncoder1.getPosition());
    System.out.println(photonCannon.getYawOfTargets());

    // System.out.println(swerve.getHeading());
    //// Get the controller axis values
    // double shooterSpeed = controller.getRawAxis(3); // Change the axis number as
    // per your setup

    // Map the controller output to motor speeds
    // double leftMotorSpeed = -shooterSpeed; // Reverse the value if necessary
    // double rightMotorSpeed = shooterSpeed;

    // Set the motor speeds :D
    // leftShooterMotor.set(leftMotorSpeed);
    // rightShooterMotor.set(rightMotorSpeed);

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
    //SmartDashboard.putNumber("Range",photonCannon.getRangeToTarget());
    //SmartDashboard.putNumber("Yaw",photonCannon.getYawOfTargets());
    photonCannon.getYawOfTargets();
    //SmartDashboard.putNumber("value", Vision.getInstance().getYawOfTargets());
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
     double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1) * ((Constants.Controllers.driver1.getRawAxis(2) + 1) / 2),
        Constants.Controllers.stickDeadband);
     double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0) * ((Constants.Controllers.driver1.getRawAxis(2) + 1) / 2),
        Constants.Controllers.stickDeadband);
    final  double rot = MathUtil.applyDeadband(-Constants.Controllers.driver1.getRawAxis(4) * ((Constants.Controllers.driver1.getRawAxis(2) + 1) / 2),
        Constants.Controllers.stickDeadband);
<<<<<<< Updated upstream
    double rot;

    if (Constants.Controllers.driver1.getRawButton(6)) {
      // while (swerve.getHeading() <= -1 || swerve.getHeading() >= 1) {

      // }
      rot = 0.1;
      System.out.println("ROTATION SET TO ZEROOOOOOOOOO");
    } else {
      rot = MathUtil.applyDeadband(-Constants.Controllers.driver1.getRawAxis(3),
          Constants.Controllers.stickDeadband);
    }
=======

        if (Constants.Controllers.driver1.getPOV() == 0) {
          xSpeed = .75;
        } else if (Constants.Controllers.driver1.getPOV() == 90) {
          ySpeed = .75;
        } else if (Constants.Controllers.driver1.getPOV() == 180) {
          xSpeed = -.75;
        } else if (Constants.Controllers.driver1.getPOV() == 270) {
          ySpeed = -.75;
        } else if (Constants.Controllers.driver1.getPOV() == 45) {
          xSpeed = .5;
          ySpeed = .5;
        } else if (Constants.Controllers.driver1.getPOV() == 135) {
          xSpeed = -.5;
          ySpeed = .5;
        } else if (Constants.Controllers.driver1.getPOV() == 225) {
          xSpeed = -.5;
          ySpeed = -.5;
        } else if (Constants.Controllers.driver1.getPOV() == 315) {
          xSpeed = .5;
          ySpeed = -.5;
        }
 
        
>>>>>>> Stashed changes

    // Drive Function
    swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
         rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);
  }

  // Should Rotate Swerve Around Target
  private void SwerveAutoAim(boolean isFieldRel) {
    final double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1),
        Constants.Controllers.stickDeadband);
    final double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0),
        Constants.Controllers.stickDeadband);

<<<<<<< Updated upstream
    final double rot;

    if (photonCannon.hasTargets()) {
      rot = -photonCannon.getYawOfTargets();
    } else {
      rot = 0;
=======
     double rot = 0;
    // while (photonCannon.hasTargets() && (photonCannon.getYawOfTargets() >= 20 || photonCannon.getYawOfTargets() <= -20)) {
    
    if (photonCannon.getYawOfTargets() >= 7 || photonCannon.getYawOfTargets() <= -7){
       rot = -photonCannon.getYawOfTargets() * 0.01;
       swerve.zeroHeading();
>>>>>>> Stashed changes
    }
//  }

    swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);
  }

}
