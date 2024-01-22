package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Climber {
    
    public static CANSparkMax masterMotor;
    public static CANSparkMax followerMotor;
    public static RelativeEncoder wristEncoder;

    public Climber() {
        masterMotor = new CANSparkMax(Constants.Climber.masterMotorID, MotorType.kBrushless);
        followerMotor = new CANSparkMax(Constants.Climber.followerMotorID, MotorType.kBrushless);
        followerMotor.follow(masterMotor);
    }

    public static void extend() {
        masterMotor.set(.25);
    }
}
