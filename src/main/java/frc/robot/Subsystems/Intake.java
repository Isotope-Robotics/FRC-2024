package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class Intake {

    public static CANSparkMax wristMotor;
    public static CANSparkMax intakeMotor;
    public static RelativeEncoder wristEncoder;
    public static DigitalInput noteIntaked;
    public static DigitalInput wristLimit;

    private static final double kP = 0.015;
    private static final double kI = 0;
    private static final double kD = 0;

    public static final PIDController wristPID = new PIDController(kP, kI, kD);

    public Intake() {
        wristMotor = new CANSparkMax(Constants.Intake.wristMotorID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.Intake.intakeMotorID, MotorType.kBrushless);
        noteIntaked = new DigitalInput(8);
        wristLimit = new DigitalInput(0);
        wristEncoder = wristMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 4096);
        wristEncoder.setPosition(0.0);
    }

    //Photoelectric Sensor for sensing a note in the intake
    public boolean getNoteIntaked() {
        if (noteIntaked.get()) {
            return true;
        } else {
            return false;
        }
    }

    //Returns Limit Switch for Wrist Limit
    public boolean getWristLimit() {
        if (wristLimit.get()) {
            return true;
        } else {
            return false;
        }
    }

    //Stops intake motors
    public void intakeStop() {
        intakeMotor.set(0);
    }

    //Stops wrist motors
    public void wristStop() {
        wristMotor.set(0);
    }

    //Wrist up movement control
    public void wristUp() {
        if (getWristLimit()) {
            wristStop();
        }
        else {
            intakeMotor.set(wristPID.calculate(wristEncoder.getPosition(), 0));
        }
    }

    //Wrist down movement control
    public void wristDown() {
        intakeMotor.set(wristPID.calculate(wristEncoder.getPosition(), 60));
    }

    //Intake speed set
    public void intakeStart(double speed) {
        intakeMotor.set(speed);
    }

}
