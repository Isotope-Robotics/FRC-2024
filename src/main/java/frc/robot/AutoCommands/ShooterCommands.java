package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Blinkin;

public class ShooterCommands {
    public static Shooter m_Shooter = Shooter.getInstance();
    public static Intake m_Intake = Intake.getInstance();
    public static Blinkin m_Blinkin = Blinkin.getInstance();

    private ShooterCommands() {
        throw new UnsupportedOperationException("This is a utility class");

    }

    // needs the yellow {} and ; for each line because multiple lines
    public static Command shoot() {
        return Commands.runOnce(() -> {
            m_Blinkin.sinelonRGB();
            m_Shooter.shoot(1.0);
        })
                .andThen(Commands.waitSeconds(0.5))
                .andThen(Commands.runOnce(() -> {
                    while (!m_Shooter.getNoteDetected()) {
                    m_Intake.intakeStart(1);
                    }
                }).andThen(Commands.waitSeconds(0.5)
                        .andThen(Commands.runOnce(() -> {
                            m_Intake.intakeStop();
                            m_Shooter.stop();
                        }))));
    }


    
    }
