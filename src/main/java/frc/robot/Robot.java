// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// e
package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Blinkin;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Vision.Limelight;
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



  // Swerve Varibles
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  // Shooter Varibles
  private final Shooter shooter = Shooter.getInstance();

  // Intake Varibles
  private final Intake intake = Intake.getInstance();

  private final Blinkin blinkin = Blinkin.getInstance();

  private final Climber climber = Climber.getInstance();

  private final Limelight limelight = new Limelight();

  public static Timer timer = new Timer();

  public static Timer hTimer = new Timer();

  public static Timer shootTimer = new Timer();

  public static boolean bop = false; // DO NOT TELL AIDEN ABOUT THIS

  public Swerve swerve;

  public boolean intook;

  // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTable limelightAprilTable = NetworkTableInstance.getDefault().getTable("limelight-april");
  NetworkTable limelightNoteTable = NetworkTableInstance.getDefault().getTable("limelight-note");

  double limelightAprilTagLastError;
  double limelightNoteLastError;

  PhotonCamera photon = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  // Photon Vision PID Setup
  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  // private double start_time = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    swerve = Swerve.getInstance();

    // Robot Container for Auto Commands
    robotContainer = new RobotContainer();

    intake.zeroEncoders();
    shooter.zeroEncoders();

    intake.clearStickyFaults();
    shooter.clearStickyFaults();
    climber.clearStickyFaults();

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



    limelight.updateLimelightData();
    
    RobotTelemetry();
    swerve.swerveCurrents();

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
    //shooter.pivotUp();
    m_AutonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_AutonomousCommand != null) {
      m_AutonomousCommand.schedule();
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    RobotTelemetry();
    swerve.swerveOdometry.update(swerve.getGyroYaw(), swerve.getModulePositions());

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // Destroy Auto Commands When Switching To TeleOP
    if (m_AutonomousCommand != null) {
      m_AutonomousCommand.cancel();
    }

    shooter.stop();

    swerve.zeroHeading();

    timer.start();
    hTimer.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
        swerve.swerveOdometry.update(swerve.getPosGyroYaw(), swerve.getModulePositions());

    Driver1Controls();
    Driver2Controls();

    SmartDashboard.putBoolean("Intook", intook);

    // AutomatedOverrides();
    RobotTelemetry();
        System.out.println("Left: " + intake.getNoteIntakedLeft() + " Mid: " + intake.getNoteIntakedMid() + " Right: " + intake.getNoteIntakedRight());

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
    // CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    swerve.swerveCurrents();
    //System.out.println("Left: " + intake.getNoteIntakedLeft() + " Mid: " + intake.getNoteIntakedMid() + " Right: " + intake.getNoteIntakedRight());
    
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  private void Driver1Controls() {
    // Back to robot centric while button seven is pushed
    if (Constants.Controllers.driver1.getRawButton(2)) {
      swerve.zeroHeading();
      System.out.println("Gyro reset");
    }

    if (Constants.Controllers.driver1.getRawButton((1))) {
      SwerveDrive(false);
      System.out.println("Robot Centric Enabled");
    } else if (Constants.Controllers.driver1.getRawButton(3)) {
      limelightAprilTagAim(true);
    } else if (Constants.Controllers.driver1.getRawButton(4)) {
      limelightNoteAim(true);
    } else if (Constants.Controllers.driver1.getRawButton(5)) {
      SwerveLock();
    } else if (Constants.Controllers.driver1.getRawButton(6)) {
      Vibrate();
    } else {
      SwerveDrive(true);
    }

    if (!Constants.Controllers.driver1.getRawButton(6)) {
      hTimer.reset();
    }

    if (!(Constants.Controllers.driver1.getRawButton(3))) {
      limelightAprilTagLastError = 0;
      // System.out.println("limelight button not pressed, setting last tx to 0");
    }
    if (!(Constants.Controllers.driver1.getRawButton(4))) {
      limelightNoteLastError = 0;
      // System.out.println("limelight button not pressed, setting last tx to 0");
    }
  }

  private void Driver2Controls() {
    // blinkin patterns
    if (intake.sens()) {
      if (shootTimer.hasElapsed(2))
      {
        
          blinkin.orange();
        }
      
      else {
        intook = true;
    blinkin.hotpink();
      }

     // System.out.println(Climber.getMasterPos());
     // System.out.println(Climber.getFollowerPos());
     // System.out.println(limelight.getLimelightPose());
    
    } else if (intake.getNoteIntakedLeft() || intake.getNoteIntakedRight()) {
    blinkin.darkBlue();
      
    intook = false;
    } else if (Constants.Controllers.driver2.getAButton() && (!intake.getNoteIntakedLeft() && !intake.getNoteIntakedRight())) {
    blinkin.rainbowRGB();
    } else {
    blinkin.colorwave();
     intook = false;
    }
    
    // Intake Manual Control
    if (Constants.Controllers.driver2.getLeftBumper() ) {
      
      intake.intakeStart(-0.75);
    } else if (Constants.Controllers.driver2.getRightBumper()) {
      intake.intakeStart(0.75);
    } else {
      intake.intakeStop();
    }

      
    if (Constants.Controllers.driver2.getXButton()) {
      Flippy();
    }

    //TODO: this is doing the same thing as the code above
    // One Button Intake
    if (Constants.Controllers.driver2.getAButton()) {
      if (intake.getNoteIntakedLeft() && intake.getNoteIntakedRight()) {
        intake.wristUp();
        intake.intakeStop();
      } else if (intake.getNoteIntakedLeft() || intake.getNoteIntakedRight() || intake.getNoteIntakedMid()) {
        intake.wristUp();
        intake.intakeStop();
      } else {
        intake.wristDown();
        intake.intakeStart(-0.75);
      }
    } else {
      intake.wristUp();
    }

    // Wrist Manual Control
    if (Constants.Controllers.driver2.getStartButton()) {
      intake.wristDown();
    } else if (Constants.Controllers.driver2.getBackButton()) {
      intake.wristHalf();
    }

    //TODO: maybe make this some auxially button???
    // one button intake half then shoot
    if (Constants.Controllers.driver2.getBButton()) {
      intake.wristHalf();
      intake.intakeStart(.1);
    }

    // One Button Shoot
    if (Constants.Controllers.driver2.getRightTriggerAxis() >= 0.10) {
      shootTimer.start();
    
       
      shooter.shoot(1.0);
      if (shootTimer.get() > 0.8 && !shooter.getNoteDetected()) {
        blinkin.orange();
      }
      if (shooter.getNoteDetected()) {
        blinkin.green();
      }
    } else if (Constants.Controllers.driver2.getLeftTriggerAxis() >= 0.10) {
        shooter.shoot(-Constants.Controllers.driver2.getLeftTriggerAxis()); // suck in
    } else {
      shootTimer.reset();
      shooter.stop();
    }
    // Climber Control
    climber.retractM(-Constants.Controllers.driver2.getRawAxis(1));
    climber.retractF(Constants.Controllers.driver2.getRawAxis(5));
  }

  private void RobotTelemetry() {

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
    
    System.out.println(intake.wristEncoder1.getPosition()); // Yes I actually do use this, David

    SmartDashboard.putNumber("Intake Current", intake.getIntakeCurrent());
    SmartDashboard.putNumber("Wrist Current", intake.getWristCurrent());

    SmartDashboard.putNumber("Master Current", climber.getMasterCurrent());

    SmartDashboard.putNumber("Follower Current", climber.getFollowerCurrent());

    SmartDashboard.putNumber("Shooter 1 Current", shooter.getShooter1Current());

    SmartDashboard.putNumber("Shooter 2 Current", shooter.getShooter2Current());

  }

  // To Drive With Controllers
  private void SwerveDrive(boolean isFieldRel) {
    // Controller Deadbands (Translation, Strafe, Rotation)

    double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1)
        * (Constants.Controllers.driver1.getRawAxis(2)),
        Constants.Controllers.stickDeadband);
    double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0)
        * (Constants.Controllers.driver1.getRawAxis(2)),
        Constants.Controllers.stickDeadband);
    double rot = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(3) //we made it unnegatived
        * (Constants.Controllers.driver1.getRawAxis(2)),
        Constants.Controllers.stickDeadband);

    // if (Constants.Controllers.driver1.getPOV() == 0) {
    // xSpeed = -.75;
    // } else if (Constants.Controllers.driver1.getPOV() == 90) {
    // ySpeed = .75;
    // } else if (Constants.Controllers.driver1.getPOV() == 180) {
    // xSpeed = .75;
    // } else if (Constants.Controllers.driver1.getPOV() == 270) {
    // ySpeed = -.75;
    // } else if (Constants.Controllers.driver1.getPOV() == 45) {
    // xSpeed = .5;
    // ySpeed = .5;
    // } else if (Constants.Controllers.driver1.getPOV() == 135) {
    // xSpeed = -.5;
    // ySpeed = .5;
    // } else if (Constants.Controllers.driver1.getPOV() == 225) {
    // xSpeed = -.5;
    // ySpeed = -.5;
    // } else if (Constants.Controllers.driver1.getPOV() == 315) {
    // xSpeed = .5;
    // ySpeed = -.5;
    // }

    // Drive Function
    swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);
  }

  private void limelightAprilTagAim(boolean isFieldRel) {
    double currentGyro = swerve.gyro.getAngle();
    double mappedAngle = 0.0f;
    double angy = ((currentGyro % 360.0f));
    if (currentGyro >= 0.0f) {
      if (angy > 180) {
        mappedAngle = angy - 360.0f;
      } else {
        mappedAngle = angy;
      }
    } else {
      if (Math.abs(angy) > 180.0f) {
        mappedAngle = angy + 360.0f;
      } else {
        mappedAngle = angy;
      }
    }
    double tx = limelightAprilTable.getEntry("tx").getFloat(700);
    // System.out.println("tx april: " + tx);
    double tx_max = 30.0f; // detemined empirically as the limelights field of view
    double error = 0.0f;
    double kP = 2.0f; // should be between 0 and 1, but can be greater than 1 to go even faster
    double kD = 0.0f; // should be between 0 and 1
    double steering_adjust = 0.0f;
    double acceptable_error_threshold = 10.0f / 360.0f; // 15 degrees allowable
    if (tx != 0.0f) { // use the limelight if it recognizes anything, and use the gyro otherwise
      error = 1.0f * (tx / tx_max) * (31.65 / 180); // scaling error between -1 and 1, with 0 being dead on, and 1
                                                     // being 180 degrees away
    } else {
      error = mappedAngle / 180.0f; // scaling error between -1 and 1, with 0 being dead on, and 1 being 180 degrees
                                    // away
    }
    if (limelightAprilTagLastError == 0.0f) {
      limelightAprilTagLastError = tx;
    }
    double error_derivative = error - limelightAprilTagLastError;
    limelightAprilTagLastError = tx; // setting limelightlasterror for next loop

    if (Math.abs(error) > acceptable_error_threshold) { // PID with a setpoint threshold
      steering_adjust = (kP * error + kD * error_derivative);
    }

    final double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1),
        Constants.Controllers.stickDeadband);
    final double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0),
        Constants.Controllers.stickDeadband);
    swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        steering_adjust * Constants.Swerve.maxAngularVelocity, isFieldRel, false);

    // System.out.println("raw angle: " + currentGyro + ", mapped angle: " +
    // mappedAngle + ", april tag error: " + error);
  }

  private void limelightNoteAim(boolean isFieldRel) {
    double tx = limelightNoteTable.getEntry("tx").getFloat(0);
    double tx_max = 30.0f; // detemined empirically as the limelights field of view
    double error = 0.0f;
    double kP = 2.0f; // should be between 0 and 1, but can be greater than 1 to go even faster
    double kD = 0.0f; // should be between 0 and 1
    double steering_adjust = 0.0f;
    double acceptable_error_threshold = 10.0f / 360.0f; // 15 degrees allowable
    error = 1.0 * (tx / tx_max) * (31.65 / 180); // scaling error between -1 and 1, with 0 being dead on, and 1 being
                                                  // 180 degrees away
    if (limelightNoteLastError == 0.0f) {
      limelightNoteLastError = tx;
    }
    double error_derivative = error - limelightNoteLastError;
    limelightNoteLastError = tx; // setting limelightlasterror for next loop

    if (Math.abs(error) > acceptable_error_threshold) { // PID with a setpoint threshold
      steering_adjust = (kP * error + kD * error_derivative);
    }

    final double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1),
        Constants.Controllers.stickDeadband);
    final double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0),
        Constants.Controllers.stickDeadband);
    swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        steering_adjust * Constants.Swerve.maxAngularVelocity, isFieldRel, false);

    // System.out.println("Note error: " + error);
  }

  // uses photon vision, we are using limelight, keeping in case the code is
  // useful later
  public void NoteAutoAim(boolean isFieldRel) {
    double steering_adjust;
    var result = photon.getLatestResult();

    if (result.hasTargets()) {
      steering_adjust = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      steering_adjust = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(3)
          * (Constants.Controllers.driver1.getRawAxis(2)),
          Constants.Controllers.stickDeadband);
    }

    final double xSpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1),
        Constants.Controllers.stickDeadband);
    final double ySpeed = MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0),
        Constants.Controllers.stickDeadband);
    swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        steering_adjust * Constants.Swerve.maxAngularVelocity, isFieldRel, false);
  }

  public void SwerveLock() {
    swerve.lock();
  }

  public void Vibrate() {
    double period = .6; //PERIOD in seconds
    if (hTimer.get() < period* 0.6) {
      swerve.forward2(false);
    } else if ((hTimer.get() >= period * 0.6) && (hTimer.get() <= period)){
      swerve.backward(false);
     } else {
       hTimer.reset();
    }
  }

  public void Flippy() {
    double period = .1; //PERIOD in seconds
    if (hTimer.get() < period* 0.6) {
      intake.wristMotor1.set(1);
    } else if ((hTimer.get() >= period * 0.6) && (hTimer.get() <= period)){
      intake.wristMotor1.set(-1);
     } else {
       hTimer.reset();
    }
  }

  private void SwerveAprilDrive(boolean isFieldRel) {
    // Controller Deadbands (Translation, Strafe, Rotation)

    // double xSpeed = drivePID.calculate(limelight.getALimelight(), driveOffset);
    // double ySpeed = strafePID.calculate(limelight.getXLimelight(), strafeOffset);
    // double rot = rotationPID.calculate(limelight.getYaw(), rotationOffset);

    // if (Constants.Controllers.driver1.getPOV() == 0) {
    // xSpeed = -.75;
    // } else if (Constants.Controllers.driver1.getPOV() == 90) {
    // ySpeed = .75;
    // } else if (Constants.Controllers.driver1.getPOV() == 180) {
    // xSpeed = .75;
    // } else if (Constants.Controllers.driver1.getPOV() == 270) {
    // ySpeed = -.75;
    // } else if (Constants.Controllers.driver1.getPOV() == 45) {
    // xSpeed = .5;
    // ySpeed = .5;
    // } else if (Constants.Controllers.driver1.getPOV() == 135) {
    // xSpeed = -.5;
    // ySpeed = .5;
    // } else if (Constants.Controllers.driver1.getPOV() == 225) {
    // xSpeed = -.5;
    // ySpeed = -.5;
    // } else if (Constants.Controllers.driver1.getPOV() == 315) {
    // xSpeed = .5;
    // ySpeed = -.5;
    // }

    // Drive Function
    // swerve.drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
    //     rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);
  }

  /*
   * private void SwerveGyro0(boolean isFieldRel) {
   * final double xSpeed =
   * MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(1),
   * Constants.Controllers.stickDeadband);
   * final double ySpeed =
   * MathUtil.applyDeadband(Constants.Controllers.driver1.getRawAxis(0),
   * Constants.Controllers.stickDeadband);
   * 
   * double rot = 0;
   * // while (photonCannon.hasTargets() && (photonCannon.getYawOfTargets() >= 20
   * ||
   * // photonCannon.getYawOfTargets() <= -20)) {
   * 
   * if (swerve.getRealYaw() >= 1 || swerve.getRealYaw() <= -1) {
   * rot = -swerve.getRealYaw() * 0.01;
   * 
   * }
   * // }
   * 
   * swerve.drive(new Translation2d(xSpeed,
   * ySpeed).times(Constants.Swerve.maxSpeed),
   * rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);
   * }
   */

}
