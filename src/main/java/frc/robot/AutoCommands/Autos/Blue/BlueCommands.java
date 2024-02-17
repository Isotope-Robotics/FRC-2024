package frc.robot.AutoCommands.Autos.Blue;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.AutoCommands.ShooterCommands;
import frc.robot.Subsystems.Swerve;

public class BlueCommands extends Command {
    public Swerve swerve = Swerve.getInstance();

    public Command DrivePastLine(){
         PathPlannerPath path = PathPlannerPath.fromPathFile("Drive To Line");

        //If this is the first path it should reset to inital pose
        swerve.swerveOdometry.resetPosition(swerve.getHeading(), swerve.getModulePositions(), path.getPreviewStartingHolonomicPose());
        

        return new FollowPathHolonomic(
                path,
                swerve::getPose, // Robot pose supplier
                swerve::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
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
                swerve // Reference to this subsystem to set requirements
        );
    }

    public Command DriveToSpeakerNote1(){
         PathPlannerPath path = PathPlannerPath.fromPathFile("Drive To Note 1 From Speaker");

        //If this is the first path it should reset to inital pose
        swerve.swerveOdometry.resetPosition(swerve.getHeading(), swerve.getModulePositions(), path.getPreviewStartingHolonomicPose());
        

        return new FollowPathHolonomic(
                path,
                swerve::getPose, // Robot pose supplier
                swerve::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
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
                swerve // Reference to this subsystem to set requirements
        );
    }

    public Command DriveFromNote1ToSpeaker(){
         PathPlannerPath path = PathPlannerPath.fromPathFile("Drive From Note 1 To Speaker");

        //If this is the first path it should reset to inital pose
        swerve.swerveOdometry.resetPosition(swerve.getHeading(), swerve.getModulePositions(), path.getPreviewStartingHolonomicPose());
        

        return new FollowPathHolonomic(
                path,
                swerve::getPose, // Robot pose supplier
                swerve::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
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
                swerve // Reference to this subsystem to set requirements
        );
    }

    public Command SpeakerShootCrossLine(){
        return Commands.sequence(ShooterCommands.shoot().andThen(Commands.sequence(DriveToSpeakerNote1())));
    }

    public Command SpeakerGrab2NoteCrossLine(){
        return Commands.sequence(ShooterCommands.shoot()
        .andThen(Commands.sequence(DriveToSpeakerNote1()))
        .andThen(DriveFromNote1ToSpeaker())
        .andThen(ShooterCommands.shoot()));
    }


}
