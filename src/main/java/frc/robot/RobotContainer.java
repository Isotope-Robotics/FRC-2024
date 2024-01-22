package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoCommands.IntakeCommands;

//THIS IS A ROBOT CONTAINER ONLY FOR AUTO PERIOD COMMANDS!!!!!!!

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // For Adding Print Statements in PathPlanner
        NamedCommands.registerCommand("DropIntake", IntakeCommands.DropToPickUp());
        NamedCommands.registerCommand("IntakeNote", IntakeCommands.IntakeNote());

        // Sets up auto buttons on SmartDashboard
        configureAutos();

        // Builds Autos from Selected Auto
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    // Adds SmartDashboard Buttons for Auto Selection
    private void configureAutos() {
        SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));
    }

    // Runs that selected command
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}