package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Lib.Util.FalconSwerveConstants;
import frc.robot.Lib.Util.SwerveModuleConstants;

public class Constants {

    // Operator Controllers Constants Class
    public static final class Controllers {
        public static final Joystick driver1 = new Joystick(0);
        public static final XboxController driver2 = new XboxController(1);

        public static final double stickDeadband = 0.3;
    }

    public static final class Blinkin {
        public static final int blinkinPort = 9;
    }

    public static final class Shooter {
        public static final int shooterMotor1ID = 40;
        public static final int shooterMotor2ID = 3;

        public static final double kP = 0.015;
        public static final double kI = 0.0;
        public static final double kD = 0.0;


        public static final IdleMode Brake = IdleMode.kBrake;
        public static final IdleMode Coast = IdleMode.kCoast;
    }

    public static final class Climber {
        public static final int masterMotorID = 42;
        public static final int followerMotorID = 43;

        public static final double kP = 0.015;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final IdleMode Brake = IdleMode.kBrake;
        public static final IdleMode Coast = IdleMode.kCoast;
    }

    public static final class Intake {
        public static final int wristMotor1ID = 44;
        public static final int intakeMotorID = 45;

        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.00001;

        public static final IdleMode Brake = IdleMode.kBrake;
        public static final IdleMode Coast = IdleMode.kCoast;
    }

    public static final class Encoders {
        public static final int NEO_ENCODER_COUNTS = 42;
        public static final int FALCON_ENCODER_COUNTS = 2048;
    }

    public static final class PhotonCannon {
        public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(10); // Change Me
        public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(10); // Change Me
        public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(20); // Change Me
    }

    // Swerve Module Constants Class
    public static final class Swerve {

        // Pigeon CAN ID
        public static final int pigeonId = 13;

        // Module Drive Ratios
        public static final FalconSwerveConstants chosenModule = FalconSwerveConstants.SDS.MK4i
                .Falcon500(FalconSwerveConstants.SDS.MK4i.driveRatios.L69);

        public static final FalconSwerveConstants module0 = FalconSwerveConstants.SDS.MK4i
                .Falcon500Inverted(FalconSwerveConstants.SDS.MK4i.driveRatios.L69);

        // Drivetrain Constants
        public static final double trackWidth = Units.inchesToMeters(29.0);
        public static final double wheelBase = Units.inchesToMeters(29.0);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        public static final Translation2d flModuleOffset = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);

        // Swerve Kinematics
        //Instead of each module being like a quadrant on a plane, the center being the origin, +x is front, +y is left, cause we did our modules counter-clockwise
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), //mod 0 front right
                //new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), //mod1 front left
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0), //mod1 front left
                //new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), //mod2 back left
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), //mod2 back left
                //new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); //mod3 back right
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); //mod3 back right

        // Gear Ratios
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        // Motor Inverts
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        // Angle Encoder Invert
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        // Swerve Current Limiting
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 50;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        // Angle Motor PID Values
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        // Drive Motor PID Values
        public static final double driveKP = 0.12;//0.08;//2.0;
        public static final double driveKI = 0.0;//0.01;//40.0;
        public static final double driveKD = 0.00;//0.02;
        public static final double driveKF = 0.0;

        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        // Swerve Profiling Values
        // Meters per Second
        public static final double maxSpeed = 5.21;
        // Radians per Second
        public static final double driveRadius = Math.hypot(wheelBase, trackWidth) / 2.0;
        public static final double maxAngularVelocity = maxSpeed / driveRadius;

        // Neutral Modes
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        // Module Specific Constants
        /* Front Right Module - Module 0  */
        public static final class Mod0 {
            public static final int driveMotorID = 37;
            public static final int angleMotorID = 27;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-25);// 0 original offset
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Left Module - Module 1  */
        public static final class Mod1 {
            public static final int driveMotorID = 38;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-80);// -45 original offset
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 39;
            public static final int angleMotorID = 18;
            public static final int canCoderID = 19;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(10);// 9 original offset
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 36;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 25;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(207);// 34.75 original offset
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        // For Pathplanner Config - DO NOT USE ANYWHERE EXCEPT FOR THE CONFIG IN AUTO!!
        public static final Translation2d front_offset = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
        
        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(8.0, 0, 0), // Translation constants kP was 5.0
                new PIDConstants(1.0, 0, 0), // Rotation constants kP was 5.0
                5.21,//maxSpeed, was 5.39
                front_offset.getNorm(), // Drive base radius (distance from center to furthest module)
                new ReplanningConfig());
    }

    
    public static final class AutoConstants {
        // public static final double kMaxSpeedMetersPerSecond = 5.39;
        // public static final double kMaxAccelerationMetersPerSecondSquared = 5;
        // public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI + 2;
        // public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI + 2;

        // public static final double kPXController = 1;
        // public static final double kPYController = 1;
        // public static final double kPThetaController = 1;

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    

}
