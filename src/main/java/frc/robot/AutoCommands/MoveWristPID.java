package frc.robot.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Intake;

public class MoveWristPID extends PIDCommand {

  public MoveWristPID(double setPoint, Intake m_Intake) {
    super(
        // The controller that the command will use
        new PIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD),
        // This should return the measurement
        () -> m_Intake.wristEncoder1.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> setPoint,
        // This uses the output
        output -> {
          m_Intake.setWrist(output);
        }
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    );
    //getController().enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
