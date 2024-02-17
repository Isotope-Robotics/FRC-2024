package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoCommands.*;
import frc.robot.AutoCommands.Autos.Blue.BlueCommands;

//THIS IS A ROBOT CONTAINER ONLY FOR AUTO PERIOD COMMANDS!!!!!!!

public class RobotContainer {
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final BlueCommands blue = new BlueCommands();

    private final Command Default_Auto = blue.DrivePastLine();
    private final Command Drive_Past_Line = blue.DrivePastLine();
    private final Command SpeakerShootCrossLine = blue.SpeakerShootCrossLine();
    private final Command SpeakerGrab2NoteCrossLine = blue.SpeakerGrab2NoteCrossLine();

    public RobotContainer() {
        // For Adding Print Statements in PathPlanner
        NamedCommands.registerCommand("DropIntake", IntakeCommands.DropToPickUp());
        NamedCommands.registerCommand("IntakeNote", IntakeCommands.IntakeNote());
        NamedCommands.registerCommand("Shoot", ShooterCommands.shoot());

        // Sets up auto buttons on SmartDashboard
        configureAutos();

        SmartDashboard.putData("Auto Modes", autoChooser);

          }

    // Adds SmartDashboard Buttons for Auto Selection
    private void configureAutos() {
        autoChooser.setDefaultOption("Default", Drive_Past_Line);
        //autoChooser.addOption("Drive Past Line", Drive_Past_Line);
        //autoChooser.addOption("Shoot 1 Note Speaker", SpeakerShootCrossLine);
        //autoChooser.addOption("Shooter 2 Notes Speaker", SpeakerGrab2NoteCrossLine);
    }

    // Runs that selected command
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
