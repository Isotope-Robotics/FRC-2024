package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Intake;

public class IntakeCommands {
    public static Intake m_Intake = Intake.getInstance();

    private IntakeCommands() {
        throw new UnsupportedOperationException("This is a utility class");
    }

    // Drop Intake
    public static Command DropToPickUp() {
        return Commands.runOnce(() -> {
            m_Intake.wristDown();
        }, m_Intake);
    }

    public static Command UpToShoot(){
        return Commands.runOnce(()-> {
            m_Intake.wristUp();
        }, m_Intake);
    }

    public static Command DropAndIntake(){
        return Commands.runOnce(()-> {
            m_Intake.wristDown();
            m_Intake.intakeStart(1.0);
        }, m_Intake).andThen(
            Commands.waitSeconds(0.50)
            .andThen(()-> {
                m_Intake.wristUp();
            }).andThen(Commands.waitSeconds(0.05))
            .andThen(Commands.runOnce(()->{
                m_Intake.intakeStop();
            })
            )
        );
    }

    // Intake Note, If Note is Intaked Then Up the Wrist to Travel
    public static Command IntakeNote() {
        return Commands.runOnce(() -> {
            m_Intake.intakeStart(-0.75);
        }, m_Intake)
                .andThen(
                        Commands.waitUntil(() -> m_Intake.getNoteIntaked()).withTimeout(0.3))
                .andThen(
                        Commands.runOnce(() -> {
                            m_Intake.intakeStop();
                        }));

    }
}