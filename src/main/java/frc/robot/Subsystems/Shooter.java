package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Shooter  {

    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;

    public Shooter() {
        shooterMotor1 = new CANSparkMax(Constants.Shooter.shooterMotor1ID, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(Constants.Shooter.shooterMotor2ID, MotorType.kBrushless);
    }

    public void shoot(double leftSpeed, double rightSpeed) {
        // Set both shooter motors to shoot (adjust the speed as needed)
        shooterMotor1.set(leftSpeed);
        shooterMotor2.set(rightSpeed);
    }

    public void stop() {
        // Stop both shooter motors
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    
}
