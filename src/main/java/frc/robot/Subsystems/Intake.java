package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

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
        wristEncoder = wristMotor.getEncoder();
        noteIntaked = new DigitalInput(8);
        wristLimit = new DigitalInput(0);
    }

    public boolean getNoteIntaked() {
        // gets whether a note has been intaken
        if (noteIntaked.get()) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getWristLimit() {
        // gets whether the wrist limitswitch is hit
        if (wristLimit.get()) {
            return true;
        } else {
            return false;
        }
    }

    public void intakeStop() {
        // Stop the intake motor
        intakeMotor.set(0);
    }

    public void wristStop() {
        // Stop moving the wrist
        wristMotor.set(0);
    }

    public void wristUp() {
        // Set wrist back up to start, if it hits the limit switch stop it
        if (getWristLimit()) {
            wristStop();
        }
        else {
            intakeMotor.set(wristPID.calculate(wristEncoder.getPosition(), 0));
        }
    }

    public void wristDown() {
        // Set wrist position down to the floor for intaking
        intakeMotor.set(wristPID.calculate(wristEncoder.getPosition(), 60));
    }

    public void intakeStart(double speed) {
        // Set the intake motor to intake off the floor (adjust the speed as needed)
        intakeMotor.set(speed);
    }

}
