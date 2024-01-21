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

    public static final PIDController wristPID = new PIDController(Constants.Intake.kP, Constants.Intake.kI,
            Constants.Intake.kD);

    public Intake(int wristMotorCANID, int intakeMotorCANID) {

        // Motor Declarations
        wristMotor = new CANSparkMax(wristMotorCANID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(intakeMotorCANID, MotorType.kBrushless);

        // Idle Mode Declarations
        wristMotor.setIdleMode(Constants.Intake.Brake);
        intakeMotor.setIdleMode(Constants.Intake.Coast);

        // Limit Switch (Photo Eye) Declarations
        noteIntaked = new DigitalInput(8);
        wristLimit = new DigitalInput(0);

        // Encoders Declarations
        wristEncoder = wristMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor,
                Constants.Encoders.NEO_ENCODER_COUNTS);

    }

    // Photoelectric Sensor for sensing a note in the intake
    public boolean getNoteIntaked() {
        if (noteIntaked.get()) {
            return true;
        } else {
            return false;
        }
    }

    // Returns Limit Switch for Wrist Limit
    public boolean getWristLimit() {
        if (wristLimit.get()) {
            return true;
        } else {
            return false;
        }
    }

    // Stops intake motors
    public void intakeStop() {
        intakeMotor.set(0);
    }

    // Stops wrist motors
    public void wristStop() {
        wristMotor.set(0);
    }

    public void zeroEncoders() {
        // Zero Out Encoder Positions
        wristEncoder.setPosition(0.0);
        System.err.println("Zeroed Intake Encoders");
    }

    // Wrist up movement control
    public void wristUp() {
        if (getWristLimit()) {
            wristStop();
        } else {
            intakeMotor.set(wristPID.calculate(wristEncoder.getPosition(), 0));
        }
    }

    // Wrist down movement control
    public void wristDown() {
        intakeMotor.set(wristPID.calculate(wristEncoder.getPosition(), 60));
    }

    // Intake speed set
    public void intakeStart(double speed) {
        intakeMotor.set(speed);
    }

}
