package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Subsystems.Vision.Limelight;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public Field2d field = new Field2d();

      NetworkTable limelightAprilTable = NetworkTableInstance.getDefault().getTable("limelight-note");
  NetworkTable limelightNoteTable = NetworkTableInstance.getDefault().getTable("limelight-april");

  double limelightAprilTagLastError;
  double limelightNoteLastError;

    private static Swerve m_Instance = null;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonId);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getSpeeds,
                this::driveRobotRelative,
                Constants.Swerve.pathFollowerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);
    }

    private Limelight limelight;

    public Swerve (Limelight limelight){
      this.limelight = limelight;
    }

    public void drive(Translation2d translation, double rotation, boolean isFieldRel, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                isFieldRel ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANCoder().getDegrees());
             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Current", mod.getDriveCurrent());
             SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Current", mod.getDriveCurrent());

        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getPosGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getPosGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(getPosGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    // Returns Gyro as a Rotation2d
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-gyro.getYaw().getValue());
    }

    public Rotation2d getPosGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    // For Telementry Info, Returns as a Double Value
    public double getRealYaw() {
        return -gyro.getYaw().getValue();
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    // Returns Instance Of Swerve
    public static Swerve getInstance() {
        if (m_Instance == null) {
            m_Instance = new Swerve();
        }
        return m_Instance;
    }

    @Override
    public void periodic() {

      
      //  swerveOdometry.update(getPosGyroYaw(), getModulePositions());

        
        field.setRobotPose(getPose());


        for (SwerveModule mod : mSwerveMods) {
            /*SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
         */   //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Current", );
        
        }
    }

    public void swerveCurrents() {
              for (SwerveModule mod : mSwerveMods) {

                  SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Current", mod.getDriveCurrent());
                                    SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Current", mod.getAngleCurrent());

              }
    }

    public boolean limelightNoteAim(boolean isFieldRel) {
      boolean closeenough = false;
    double tx = limelightNoteTable.getEntry("tx").getFloat(0);
    double tx_max = 30.0f; // detemined empirically as the limelights field of view
    double error = 0.0f;
    double kP = 0.6f; // should be between 0 and 1, but can be greater than 1 to go even faster
    double kD = 0.0f; // should be between 0 and 1
    double steering_adjust = 0.0f;
    double acceptable_error_threshold = 7.0f / 360.0f; // 15 degrees allowable
    error = (tx / tx_max) * (31.65 / 180); // scaling error between -1 and 1, with 0 being dead on, and 1 being 180 degrees away
    if (limelightNoteLastError == 0.0f) {
      limelightNoteLastError = tx;
    }
    double error_derivative = error - limelightNoteLastError;
    limelightNoteLastError = tx; // setting limelightlasterror for next loop

    if (Math.abs(error) > acceptable_error_threshold) { // PID with a setpoint threshold
      steering_adjust = -1 * (kP * error + kD * error_derivative);
      closeenough = false;
    }
    else {closeenough = true;}

    final double xSpeed = 0;
    final double ySpeed = 0;
    drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        steering_adjust * Constants.Swerve.maxAngularVelocity, isFieldRel, false);

    //System.out.println("Note error: " + error);
   return closeenough;
  }

  public void forward(boolean isFieldRel) {
    
    final double xSpeed = -0.7;
    final double ySpeed = 0;
    final double rot = 0;
    drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);

  }

  public void backward(boolean isFieldRel) {
    
    final double xSpeed = 1;
    final double ySpeed = 0;
    final double rot = 0;
    drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);

  }

   public void forward2(boolean isFieldRel) {
    
    final double xSpeed = -1;
    final double ySpeed = 0;
    final double rot = 0;
    drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        rot * Constants.Swerve.maxAngularVelocity, isFieldRel, false);

  }

  public void limelightAprilTagAim(boolean isFieldRel) {
    double currentGyro = gyro.getAngle();
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
    //System.out.println("tx april: " + tx);
    double tx_max = 30.0f; // detemined empirically as the limelights field of view
    double error = 0.0f;
    double kP = 2.0f; // should be between 0 and 1, but can be greater than 1 to go even faster
    double kD = 0.0f; // should be between 0 and 1
    double steering_adjust = 0.0f;
    double acceptable_error_threshold = 10.0f / 360.0f; // 15 degrees allowable
    if (tx != 0.0f) { // use the limelight if it recognizes anything, and use the gyro otherwise
      error = -1.0f * (tx / tx_max) * (31.65 / 180); // scaling error between -1 and 1, with 0 being dead on, and 1
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

    final double xSpeed = 0;
    final double ySpeed = 0;
    drive(new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed),
        steering_adjust * Constants.Swerve.maxAngularVelocity, isFieldRel, false);

    //System.out.println("raw angle: " + currentGyro + ", mapped angle: " + mappedAngle + ", april tag error: " + error);
  }

  Rotation2d swr = new Rotation2d(45);
  Rotation2d swr2 = new Rotation2d(-45);

  SwerveModuleState sw = new SwerveModuleState(0.0, swr);
    SwerveModuleState sw2 = new SwerveModuleState(0.0, swr2);


  public void lock() {
    for (SwerveModule mod : mSwerveMods) {
      if (mod.moduleNumber == 1 || mod.moduleNumber == 3) {
mod.setDesiredState(sw, false);
      } else {
        mod.setDesiredState(sw2, false);

      }
              }
  }


}
