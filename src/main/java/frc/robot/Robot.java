// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// e
package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Vision.Vision;

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

  private double start_time = 0;



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
     // intake.wristDown();
      intake.intakeStart(-0.5);
    } else if (Constants.Controllers.driver2.getRightBumper() && intake.getNoteIntaked()) {
      intake.intakeStart(0.5);
    } else {

      intake.intakeStop();
    }

//one button intake
    if (Constants.Controllers.driver2.getPOV() == 270) {
      intake.wristDown();
      intake.intakeStart(1.0);
} if (intake.getWristLimit() == true) {
  intake.intakeStart(-1.0);
}


//one button shoot
    if(Constants.Controllers.driver2.getStartButton()){
      start_time = System.currentTimeMillis();
    } else if(System.currentTimeMillis() - start_time > 0.2){
      shooter.shoot(1.0);
    } else if(System.currentTimeMillis() - start_time < 0.2 && System.currentTimeMillis() - start_time > 0.4){
      if(intake.getNoteIntaked() || shooter.getNoteDetected()){
        shooter.shoot(1.0);
        intake.intakeStart(-1.0);
      }
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



    

    if (Constants.Controllers.driver2.getYButton() && shooter.getNoteDetected()) {
      shooter.shoot(1.0);
    } else if (Constants.Controllers.driver2.getXButton()) {
      shooter.shoot(-1.0 * Constants.Controllers.driver2.getRightTriggerAxis());
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

    if (Constants.Controllers.driver1.getRawButton(5)) {
      SwerveGyro0(true);
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

     double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1)
      * ((Constants.Controllers.driver1.getRawAxis(2) + 1) / 2),
        Constants.Controllers.stickDeadband);
     double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0)
      * ((Constants.Controllers.driver1.getRawAxis(2) + 1) / 2),
        Constants.Controllers.stickDeadband);
      double rot = MathUtil.applyDeadband(-Constants.Controllers.driver1.getRawAxis(3)
     * ((Constants.Controllers.driver1.getRawAxis(2) + 1) / 2),
        Constants.Controllers.stickDeadband);

        if (Constants.Controllers.driver1.getPOV() == 0) {
          xSpeed = -.75;
        } else if (Constants.Controllers.driver1.getPOV() == 90) {
          ySpeed = .75;
        } else if (Constants.Controllers.driver1.getPOV() == 180) {
          xSpeed = .75;
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

     double rot = 0;
    // while (photonCannon.hasTargets() && (photonCannon.getYawOfTargets() >= 20 || photonCannon.getYawOfTargets() <= -20)) {
    
    if (photonCannon.getYawOfTargets() >= 7 || photonCannon.getYawOfTargets() <= -7){
       rot = -photonCannon.getYawOfTargets() * 0.01;
       //swerve.zeroHeading();
    } else {
      rot = MathUtil.applyDeadband(-Constants.Controllers.driver1.getRawAxis(3)
     * ((Constants.Controllers.driver1.getRawAxis(2) + 1) / 2),
        Constants.Controllers.stickDeadband);
    }
// }

    swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);
  }


  //   private void limelightAim(boolean isFieldRel) {
  //     final double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1),
  //       Constants.Controllers.stickDeadband);
  //   final double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0),
  //       Constants.Controllers.stickDeadband);
  //   // Replace "limelight" with name of your Limelight table
  //   var limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  //   // Get Limelight data
  //   double rot = limelightTable.getEntry("ty").getDouble(0.0);

  //   swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
  //   rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);
    
  //     if (limelightAim(isFieldRel);. >= 7 || limelightTable.getYawOfTargets() <= -7){
  //       rot = -limelightAim.getYawOfTargets()* 0.01;

  //     }
  // }



 private void SwerveGyro0(boolean isFieldRel) {
    final double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1),
        Constants.Controllers.stickDeadband);
    final double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0),
        Constants.Controllers.stickDeadband);

     double rot = 0;
    // while (photonCannon.hasTargets() && (photonCannon.getYawOfTargets() >= 20 || photonCannon.getYawOfTargets() <= -20)) {
    
    if (swerve.getRealYaw() >= 1 || swerve.getRealYaw() <= -1){
       rot = -swerve.getRealYaw() * 0.01;
       
    }
//  }

    swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);
  }

}
