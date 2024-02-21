package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class WristUp extends Command{
    private final Intake m_Intake;

    public WristUp(Intake intake){
        m_Intake = intake;
        addRequirements(m_Intake);
    }

    @Override
    public void initialize() {
        m_Intake.wristUp();
    }

    @Override
    public boolean isFinished(){
        return m_Intake.wristPID.atSetpoint();
    }
}
