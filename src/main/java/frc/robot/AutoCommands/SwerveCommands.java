package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve;

public class SwerveCommands {


        public static Swerve m_Swerve = Swerve.getInstance();
        public static Intake m_Intake = Intake.getInstance();


    public static Command NoteAutoAim() {
        return Commands.runOnce(() -> {
            while(!m_Swerve.limelightNoteAim(true)) {}
        }, m_Swerve)
                .andThen(
                        Commands.waitSeconds(.5))
                .andThen(
                        Commands.runOnce(() -> {
                            m_Swerve.forward(false); 
                            m_Intake.intakeStart(-1);
                        }, m_Swerve).andThen(Commands.waitSeconds(.5))
                //do we need to stop going forwards here?
                .andThen(
                        Commands.runOnce(() -> {
                            m_Swerve.lock(); 
                        }, m_Swerve).andThen(Commands.waitSeconds(.5))
                .andThen(
                        Commands.runOnce(() -> {
                            while (!m_Intake.getNoteIntakedLeft() && !m_Intake.getNoteIntakedRight()) {
                                                          //  m_Intake.intakeStart(-1);

                            }
                            m_Intake.intakeStop(); 
                        }, m_Intake))));

    }

    public static Command AprTagAutoAim() {
        return Commands.run(() -> {
            m_Swerve.limelightAprilTagAim(true);
        }, m_Swerve)
                .andThen(
                        Commands.waitSeconds(1))
                .andThen(
                        Commands.run(() -> {
                            m_Swerve.forward(false);
                        }, m_Swerve).andThen(Commands.waitSeconds(.25)));

    }

  


    
    
}
