package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;

    public Shooter() {
    }

    public void shoot() {
        // Set both shooter motors to shoot (adjust the speed as needed)
        shooterMotor1.set(0.8);
        shooterMotor2.set(0.8);
    }

    public void stopShooter() {
        // Stop both shooter motors
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    @Override
    public void periodic() {
        // The periodic method is called every loop, put code here that you want to run periodically
    }
}
