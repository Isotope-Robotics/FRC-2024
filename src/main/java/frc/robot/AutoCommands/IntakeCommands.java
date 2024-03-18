package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Intake;

public class IntakeCommands {
    public static Intake m_Intake = Intake.getInstance();

    public IntakeCommands() {

    }

    // Drop Intake
    public Command DropToPickUp() {
        return Commands.run(() -> {
            m_Intake.wristDown();
        }, m_Intake);

    };

    public Command UpToShoot() {
        return Commands.run(() -> {
            m_Intake.wristUp();
        }, m_Intake);
    }

    public static Command DropAndIntake() {
        return Commands.runOnce(() -> {
            m_Intake.wristDown();
            m_Intake.intakeStart(1.0);
        }, m_Intake).andThen(
                Commands.waitSeconds(0.30)
                        .andThen(() -> {
                            m_Intake.wristUp();
                        }).andThen(Commands.waitSeconds(0.05))
                        .andThen(Commands.runOnce(() -> {
                            m_Intake.intakeStop();
                        })));
    }

    // Intake Note, If Note is Intaked Then Up the Wrist to Travel
    public static Command IntakeNote() {
        return Commands.run(() -> {
            m_Intake.intakeStart(-1);
        }, m_Intake)
                .andThen(
                        Commands.waitSeconds(1))
                .andThen(
                        Commands.runOnce(() -> {
                            m_Intake.intakeStop();
                        }));

    }

    public static Command stopIntake() {
        return Commands.runOnce(() -> {
                while (m_Intake.getNoteIntakedLeft() && m_Intake.getNoteIntakedRight()) {

            m_Intake.intakeStop(); }
        });
    }

    public static Command StopWrist() {
        return Commands.runOnce(() -> {
            m_Intake.wristStop();
        });
    }

    public static Command IntakeLess() {
        return Commands.runOnce(() -> {
            m_Intake.intakeStart(-1);
        }, m_Intake)
        .andThen(
                        Commands.waitSeconds(0.5))
                .andThen(
                        Commands.runOnce(() -> {
                            m_Intake.intakeStop();
                        }));
    }
}