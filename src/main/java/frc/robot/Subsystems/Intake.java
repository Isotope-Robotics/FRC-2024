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

    public static final PIDController wristPID = new PIDController(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD);

    public Intake() {
        //Motor Declarations
        wristMotor = new CANSparkMax(Constants.Intake.wristMotorID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.Intake.intakeMotorID, MotorType.kBrushless);

        //Idle Mode Declarations
        wristMotor.setIdleMode(Constants.Intake.Brake);
        intakeMotor.setIdleMode(Constants.Intake.Coast);

        //Limit Switch (Photo Eye) Declarations
        noteIntaked = new DigitalInput(8);
        wristLimit = new DigitalInput(0);

        //Encoders Declarations
        wristEncoder = wristMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 4096);

        //Zero Out Encoder Positions
        wristEncoder.setPosition(0.0);
    }

    //Photoelectric Sensor for sensing a note in the intake
    public boolean getNoteIntaked() {
        // gets whether a note has been intaken
        if (noteIntaked.get()) {
            return true;
        } else {
            return false;
        }
    }

    //Returns Limit Switch for Wrist Limit
    public boolean getWristLimit() {
        // gets whether the wrist limitswitch is hit
        if (wristLimit.get()) {
            return true;
        } else {
            return false;
        }
    }

    //Stops intake motors
    public void intakeStop() {
        // Stop the intake motor
        intakeMotor.set(0);
    }

    //Stops wrist motors
    public void wristStop() {
        // Stop moving the wrist
        wristMotor.set(0);
    }

    //Wrist up movement control
    public void wristUp() {
        // Set wrist back up to start, if it hits the limit switch stop it
        if (getWristLimit()) {
            wristStop();
        }
        else {
            intakeMotor.set(wristPID.calculate(wristEncoder.getPosition(), 0));
        }
    }

    //Wrist down movement control
    public void wristDown() {
        // Set wrist position down to the floor for intaking
        intakeMotor.set(wristPID.calculate(wristEncoder.getPosition(), 60));
    }

    //Intake speed set
    public void intakeStart(double speed) {
        // Set the intake motor to intake off the floor (adjust the speed as needed)
        intakeMotor.set(speed);
    }

}
