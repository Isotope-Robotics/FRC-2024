package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Blinkin;
import edu.wpi.first.wpilibj.Timer;

public class ShooterCommands {
    public static Shooter m_Shooter = Shooter.getInstance();
    public static Intake m_Intake = Intake.getInstance();
    public static Blinkin m_Blinkin = Blinkin.getInstance();

    public static Timer timer = new Timer();

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
                    timer.restart();
                    while (!m_Shooter.getNoteDetected() && !timer.hasElapsed(1)) {
                    m_Intake.intakeStart(1);
                    }
                }).andThen(Commands.waitSeconds(0.2)
                        .andThen(Commands.runOnce(() -> {
                         //   m_Intake.intakeStop();
                           // m_Shooter.stop();
                        }))));
    }

    public static Command shoot2() {
        return Commands.runOnce(() -> {
            m_Blinkin.sinelonRGB();
            m_Shooter.shoot(1.0);
        })
                .andThen(Commands.waitSeconds(0))
                .andThen(Commands.runOnce(() -> {
                    timer.restart();
                    while (!m_Shooter.getNoteDetected() && !timer.hasElapsed(1)) {
                    m_Intake.intakeStart(1);
                    }
                }).andThen(Commands.waitSeconds(0.2)
                        .andThen(Commands.runOnce(() -> {
                         //   m_Intake.intakeStop();
                           // m_Shooter.stop();
                        }))));
    }


    
    }
