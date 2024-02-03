package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Shooter;

public class ShooterCommands {
    public static Shooter m_Shooter = Shooter.getInstance();

    private ShooterCommands(){
        throw new UnsupportedOperationException("This is a utility class");

        
    }

    // why does this not work :(
    public static Command Shoot() {
            return Commands.run(() -> {
                m_Shooter.shoot(0.0, 0.0);
               // waitCommand(0.5);
            }, m_Shooter);
        }

        public static Command DropToPickUp() {
            return Commands.run(() -> {
                m_Shooter.wristDown();
            }, m_Shooter);
        }

   


 

}