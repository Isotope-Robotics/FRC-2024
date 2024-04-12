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
    public static RelativeEncoder masterEncoder;
    public static RelativeEncoder followerEncoder;
    public static DigitalInput climberSwitch1;
    public static DigitalInput climberSwitch2;

    private static Climber m_Instance = null;

    public static final PIDController motionPID = new PIDController(Constants.Climber.kP, Constants.Climber.kI,
            Constants.Climber.kD);

    public Climber(int masterMotorID, int followerMotorID) {
        // Motor Declarations
        masterMotor = new CANSparkMax(masterMotorID, MotorType.kBrushless);
        followerMotor = new CANSparkMax(followerMotorID, MotorType.kBrushless);
        // Idle Mode Declarations
        masterMotor.setIdleMode(Constants.Intake.Brake);
        followerMotor.setIdleMode(Constants.Intake.Brake);

        masterEncoder = masterMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, Constants.Encoders.NEO_ENCODER_COUNTS);
        followerEncoder = followerMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, Constants.Encoders.NEO_ENCODER_COUNTS);
    }


    // Climbers go up
    public void extendM(double s) {
        masterMotor.set(s);
    }

    public void extendF(double s) {
        followerMotor.set(s);

    }

    // Climbers go down
    public void retractM(double s) { // s being the stick axis
        if (Math.abs(s) < 0.1) { // deadzone
            s = 0;
        }
        masterMotor.set(s);
    }

    public void retractF(double s) {
        if (Math.abs(s) < 0.1) {
            s = 0;
        }
        followerMotor.set(s);
    }

    public void stopM() {
        masterMotor.set(0);
    }

    public void stopF() {
    followerMotor.set(0);
    }

    public void FUp() {
    followerMotor.set(motionPID.calculate(followerEncoder.getPosition(), 10.0));
    }

    public void MUp() {
    masterMotor.set(motionPID.calculate(masterEncoder.getPosition(), 10.0));
    }

    public void MDown() {
    masterMotor.set(motionPID.calculate(masterEncoder.getPosition(), 0.0));
    }

    public void FDown() {
    followerMotor.set(motionPID.calculate(followerEncoder.getPosition(), 0.0));
    }

    public void clearStickyFaults(){
        masterMotor.clearFaults();
        followerMotor.clearFaults();
        masterEncoder.setPosition(0);
        followerEncoder.setPosition(0);
        System.out.println("Clearing Climber Faults, If Any");
    }

    public  double getMasterCurrent() {
        return masterMotor.getOutputCurrent();
    }

    public  double getFollowerCurrent() {
        return followerMotor.getOutputCurrent();
    }

    public static double getFollowerPos() {
        return followerEncoder.getPosition();
    }

    public static double 
    
    getMasterPos() {
        return masterEncoder.getPosition();
    }

    // Returns Instance Of Climber
    public static Climber getInstance() {
        if (m_Instance == null) {
            m_Instance = new Climber(Constants.Climber.masterMotorID, Constants.Climber.followerMotorID);
        }

        return m_Instance;
    }
}
