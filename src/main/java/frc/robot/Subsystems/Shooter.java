package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder;

import frc.robot.Constants;

public class Shooter  {

    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    public static RelativeEncoder shooter1Encoder;
    public static RelativeEncoder shooter2Encoder;

    public Shooter() {
        //Motor Declarations
        shooterMotor1 = new CANSparkMax(Constants.Shooter.shooterMotor1ID, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(Constants.Shooter.shooterMotor2ID, MotorType.kBrushless);

        //Idle Mode Declarations
        shooterMotor1.setIdleMode(Constants.Shooter.Brake);
        shooterMotor2.setIdleMode(Constants.Shooter.Brake);

        //Encoders Declarations
        shooter1Encoder = shooterMotor1.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 4096);
        shooter2Encoder = shooterMotor2.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 4096);

        //Zero Out Encoder Positions
        shooter1Encoder.setPosition(0);
        shooter2Encoder.setPosition(0);
    }

    //Set both shooter motors to shoot (adjust the speed as needed)
    public void shoot(double leftSpeed, double rightSpeed) {
        shooterMotor1.set(leftSpeed);
        shooterMotor2.set(rightSpeed);
        SmartDashboard.putNumber("Shooter 1 Speed", leftSpeed);
        SmartDashboard.putNumber("Shooter 2 Speed", rightSpeed);
    }

    //Varible shooter control loop
    public void variableShooter(double motor1Speed, double motor2Speed){
        shooterMotor1.set(motor1Speed);
        shooterMotor2.set(motor2Speed);

        SmartDashboard.putNumber("Shooter 1 Speed", motor1Speed);
        SmartDashboard.putNumber("Shooter 2 Speed", motor2Speed);
    }

    //Stop both shooter motors
    public void stop() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
        SmartDashboard.putNumber("Shooter is Stopped", 0);
    }

    public void sourceIntake(double speed) {
        // Set both shooter motors to intake through the source (adjust the speed as needed)
        shooterMotor1.set(-speed);
        shooterMotor2.set(-speed);
    }
}
