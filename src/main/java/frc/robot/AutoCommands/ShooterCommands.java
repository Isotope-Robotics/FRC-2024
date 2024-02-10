package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Subsystems.Shooter;

public class ShooterCommands {
    public static Shooter m_Shooter = Shooter.getInstance();

    private ShooterCommands() {
        throw new UnsupportedOperationException("This is a utility class");

    }

    // needs the yellow {} and ; for each line because multiple lines
    public static Command shoot() {
        return Commands.runOnce(() -> {
            m_Shooter.shoot(.25);
            Commands.waitSeconds(.5);
            m_Shooter.shoot(0);
        });
    }

}