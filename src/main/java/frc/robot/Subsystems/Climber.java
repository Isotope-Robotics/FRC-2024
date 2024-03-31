package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Climber {

    public static CANSparkMax masterMotor;
    public static CANSparkMax followerMotor;
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

    public void stopM(){
        masterMotor.set(0);
    }

    public void stopF() {
    followerMotor.set(0);

    }

    public void clearStickyFaults(){
        masterMotor.clearFaults();
        followerMotor.clearFaults();
        System.out.println("Clearing Shooter Faults, If Any");
    }

    public  double getMasterCurrent() {
        return masterMotor.getOutputCurrent();
    }

    public  double getFollowerCurrent() {
        return followerMotor.getOutputCurrent();
    }

    // Returns Instance Of Climber
    public static Climber getInstance() {
        if (m_Instance == null) {
            m_Instance = new Climber(Constants.Climber.masterMotorID, Constants.Climber.followerMotorID);
        }

        return m_Instance;
    }
}
