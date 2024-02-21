package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoCommands.*;
import frc.robot.AutoCommands.Autos.Blue.BlueCommands;

//THIS IS A ROBOT CONTAINER ONLY FOR AUTO PERIOD COMMANDS!!!!!!!

public class RobotContainer {
    SendableChooser<Command> autoChooser;

    private final BlueCommands blue = new BlueCommands();

    private final Command DRIVE_LINE_AUTO = blue.Speaker2NoteAuto();


    public RobotContainer() {
        // For Adding Print Statements in PathPlanner
        NamedCommands.registerCommand("DropIntake", IntakeCommands.DropToPickUp());
        NamedCommands.registerCommand("UpIntake", IntakeCommands.UpToShoot());
        NamedCommands.registerCommand("IntakeNote", IntakeCommands.IntakeNote());
        NamedCommands.registerCommand("Shoot", ShooterCommands.shoot());
        NamedCommands.registerCommand("IntakeStop", IntakeCommands.stopIntake());
        NamedCommands.registerCommand("IntakeLess", IntakeCommands.IntakeLessNote());
        NamedCommands.registerCommand("StopWrist", IntakeCommands.StopWrist());

        // Sets up auto buttons on SmartDashboard
        configureAutos();

        //SmartDashboard.putData("Auto Modes", autoChooser);
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);

          }

    // Adds SmartDashboard Buttons for Auto Selection
    private void configureAutos() {
        
    }

    // Runs that selected command
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
