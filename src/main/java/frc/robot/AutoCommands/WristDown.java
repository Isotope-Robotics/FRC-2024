package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class WristDown extends Command{
    private final Intake m_Intake;

    public WristDown(Intake intake){
        m_Intake = intake;
        addRequirements(m_Intake);
    }

    @Override
    public void initialize() {
        m_Intake.wristDown();
    }

    @Override
    public boolean isFinished(){
        return Intake.wristPID.atSetpoint();
    }
}
