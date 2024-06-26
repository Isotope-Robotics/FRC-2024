package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.Blinkin;

public class SwerveCommands {


        public static Swerve m_Swerve = Swerve.getInstance();
        public static Intake m_Intake = Intake.getInstance();
        public static Blinkin m_Blinkin = Blinkin.getInstance();
        public static boolean tooLong = false;
        public static Timer timer = new Timer();

    // public static Command schizo4Note() {
    //     return Commands.runOnce(() -> {
    //         //lock on while driving forwards until note is intaken or timer is hit
    //         while(!m_Swerve.schizoNoteAim(timer)) {}
    //     });
    // }

    public static Command NoteAutoAim() {
        return Commands.runOnce(() -> {
            timer.restart();
            tooLong = false;
            while(!m_Swerve.limelightNoteAim(true) && !tooLong && (!m_Intake.getNoteIntakedLeft() || !m_Intake.getNoteIntakedRight())) {
                if (timer.hasElapsed(.7)) {
                                        tooLong = true;
                                    }
            }
        }, m_Swerve)
                .andThen(
                        Commands.waitSeconds(.05))
                .andThen(
                        Commands.runOnce(() -> {
                            timer.reset();
                            tooLong = false;
                            m_Swerve.forward(false); 
                            m_Intake.intakeStart(-1);
                        }, m_Swerve).andThen(Commands.waitSeconds(.35))
                //do we need to stop going forwards here?
                .andThen(
                        Commands.runOnce(() -> {
                            m_Swerve.lock(); 
                        }, m_Swerve).andThen(Commands.waitSeconds(.01))
                .andThen(
                        Commands.runOnce(() -> {
                            timer.start();
                            while ((!m_Intake.getNoteIntakedLeft() && !m_Intake.getNoteIntakedRight()) && !tooLong) {
                                                          //  m_Intake.intakeStart(-1);
                                    if (timer.hasElapsed(.2)) {
                                        tooLong = true;
                                    }
                            }
                            m_Blinkin.hotpink();
                           // m_Intake.intakeStop(); 
                            tooLong = false;
                        }, m_Intake))));

    }

    public static Command LessNoteAutoAim() {
        timer.reset();
        return Commands.runOnce(() -> {
            m_Intake.intakeStart(-1);
                            timer.reset();
                            tooLong = false;
                            m_Swerve.forward(false); 
                        }, m_Intake).andThen(Commands.waitSeconds(.15))
                //do we need to stop going forwards here?
                .andThen(
                        Commands.runOnce(() -> {
                            m_Swerve.lock(); 
                        }, m_Swerve).andThen(Commands.waitSeconds(.01))
                .andThen(
                        Commands.runOnce(() -> {
                            timer.start();
                            while ((!m_Intake.getNoteIntakedLeft() && !m_Intake.getNoteIntakedRight()) && !tooLong) {
                                                          //  m_Intake.intakeStart(-1);
                                    if (timer.hasElapsed(0.1)) {
                                        tooLong = true;
                                    }
                            }
                            m_Blinkin.hotpink();
                           // m_Intake.intakeStop(); 
                            tooLong = false;
                        }, m_Intake)));

    }

    public static Command FakeAutoAim() {
        timer.reset();
        return 
                        Commands.runOnce(() -> {
                            m_Swerve.forward(false); 
                            m_Intake.intakeStart(-0.8);
                        }, m_Swerve).andThen(Commands.waitSeconds(.45))
                //do we need to stop going forwards here?
                .andThen(
                        Commands.runOnce(() -> {
                            m_Swerve.lock(); 
                        }, m_Swerve).andThen(Commands.waitSeconds(.05))
                .andThen(
                        Commands.runOnce(() -> {
                            timer.start();
                            while ((!m_Intake.getNoteIntakedLeft() && !m_Intake.getNoteIntakedRight()) && !tooLong) {
                                                          //  m_Intake.intakeStart(-1);
                                    if (timer.hasElapsed(1)) {
                                        tooLong = true;
                                    }
                            }
                            m_Blinkin.hotpink();
                            m_Intake.intakeStop(); 
                            tooLong = false;
                        }, m_Intake)));

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

    public static Command Hammer() {
        return Commands.run(() -> {
            m_Swerve.forward(false);
        }, m_Swerve)
                .andThen(
                        Commands.waitSeconds(.5))
                .andThen(
                        Commands.run(() -> {
                            m_Swerve.backward(false);
                        }, m_Swerve).andThen(Commands.waitSeconds(.3)));
        }
    }

  


    
    

