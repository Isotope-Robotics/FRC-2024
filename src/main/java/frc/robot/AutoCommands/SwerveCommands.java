package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve;

public class SwerveCommands {


        public static Swerve m_Swerve = Swerve.getInstance();


    public static Command NoteAutoAim() {
        return Commands.run(() -> {
            m_Swerve.limelightNoteAim(true);
        }, m_Swerve)
                .andThen(
                        Commands.waitSeconds(0.10))
                .andThen(
                        Commands.runOnce(() -> {
                      //      m_Swerve.intakeStop();
                        }));

    }
    
    
}
