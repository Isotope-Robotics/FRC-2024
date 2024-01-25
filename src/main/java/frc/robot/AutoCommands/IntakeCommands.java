package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Intake;

public class IntakeCommands {
    public static Intake m_Intake = Intake.getInstance();

    private IntakeCommands() {
        throw new UnsupportedOperationException("This is a utility class");
    }

    // Drop Intake
    public static Command DropToPickUp() {
        return Commands.run(() -> {
            m_Intake.wristDown();
        }, m_Intake);
    }

    // Intake Note, If Note is Intaked Then Up the Wrist to Travel
    public static Command IntakeNote() {
        return Commands.runOnce(() -> {
            m_Intake.intakeStart(1.0);
        }, m_Intake)
                .andThen(
                        Commands.waitUntil(() -> m_Intake.getNoteIntaked()).withTimeout(0.5))
                .andThen(
                        Commands.runOnce(() -> {
                            m_Intake.intakeStop();
                            m_Intake.wristUp();
                        }));

    }
}