package frc.robot.AutoCommands;


import java.util.List;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve;

public class DrivetrainCommands extends Command {
    public Swerve swerve = Swerve.getInstance();

    public Command followPathCommand(String pathName, boolean isFirstPath) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        //If this is the first path it should reset to inital pose
        if (isFirstPath){
            swerve.swerveOdometry.resetPosition(swerve.getHeading(), swerve.getModulePositions(), path.getPreviewStartingHolonomicPose());
        }
        
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
}
