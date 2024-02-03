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
    public static DigitalInput climberSwitch1;
    public static DigitalInput climberSwitch2;

    private static Climber m_Instance = null;

    public static final PIDController motionPID = new PIDController(Constants.Climber.kP, Constants.Climber.kI,
            Constants.Climber.kD);

    public Climber(int masterMotorID, int followerMotorID) {
        // Motor Declarations
        masterMotor = new CANSparkMax(masterMotorID, MotorType.kBrushless);
        followerMotor = new CANSparkMax(followerMotorID, MotorType.kBrushless);
        // Follow Declaration
        followerMotor.follow(masterMotor);
        // Idle Mode Declarations
        masterMotor.setIdleMode(Constants.Intake.Brake);
        followerMotor.setIdleMode(Constants.Intake.Brake);

        climberSwitch1 = new DigitalInput(8);
        climberSwitch2 = new DigitalInput(3);
    }

    public boolean getmagnet() {
        if (climberSwitch1.get()) {
            return true;
        } else {
            return false;
        }
    }

    // Climbers go up
    public static void extend() {
        masterMotor.set(motionPID.calculate(wristEncoder.getPosition(), 60));
    }

    // Climbers go down
    public static void retract() {
        masterMotor.set(motionPID.calculate(wristEncoder.getPosition(), 0));
    }

    // Returns Instance Of Climber
    public static Climber getInstance(){
        if (m_Instance == null){
            m_Instance = new Climber(Constants.Climber.masterMotorID, Constants.Climber.followerMotorID);
        }

        return m_Instance;
    }
}
