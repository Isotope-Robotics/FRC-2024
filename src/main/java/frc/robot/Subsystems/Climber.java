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

    private static Climber m_Instance = null;

    public Climber(int masterMotorID, int followerMotorID) {
        masterMotor = new CANSparkMax(masterMotorID, MotorType.kBrushless);
        followerMotor = new CANSparkMax(followerMotorID, MotorType.kBrushless);
        followerMotor.follow(masterMotor);
        // Idle Mode Declarations
        masterMotor.setIdleMode(Constants.Intake.Brake);
        followerMotor.setIdleMode(Constants.Intake.Brake);
    }

    public static void extend() {
        masterMotor.set(0.25);
    }

    public static void retract() {
        masterMotor.set(-0.25);
    }

    public static Climber getInstance(){
        if (m_Instance == null){
            m_Instance = new Climber(Constants.Climber.masterMotorID, Constants.Climber.followerMotorID);
        }

        return m_Instance;
    }
}
