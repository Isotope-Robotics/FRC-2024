package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public static CANSparkMax wristMotor1;
    // public static CANSparkMax wristMotor2;
    public static CANSparkMax intakeMotor;
    public RelativeEncoder wristEncoder1;
    // public static RelativeEncoder wristEncoder2;
    public static DigitalInput noteIntaked;
    public static DigitalInput wristLimit;

    private static Intake m_Instance = null;

    public static final PIDController wristPID = new PIDController(Constants.Intake.kP, Constants.Intake.kI,
            Constants.Intake.kD);

    private final Blinkin blinkin = Blinkin.getInstance();

    public Intake(int wristMotor1CANID, int intakeMotorCANID) {

        // Motor Declarations
        wristMotor1 = new CANSparkMax(wristMotor1CANID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(intakeMotorCANID, MotorType.kBrushless);

        // Idle Mode Declarations
        wristMotor1.setIdleMode(Constants.Intake.Brake);
        intakeMotor.setIdleMode(Constants.Intake.Brake);

        // Limit Switch (Photo Eye) Declarations
        noteIntaked = new DigitalInput(1);
        wristLimit = new DigitalInput(5);

        // Encoders Declarations
        wristEncoder1 = wristMotor1.getEncoder(SparkRelativeEncoder.Type.kHallSensor,
                Constants.Encoders.NEO_ENCODER_COUNTS);
        // Constants.Encoders.NEO_ENCODER_COUNTS);

        // intakeMotor.burnFlash();

    }

    // Photoelectric Sensor for sensing a note in the intake
    public boolean getNoteIntaked() {
        if (!noteIntaked.get()) {
            blinkin.rainbowForest();
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
        blinkin.breathRed();
    }

    // Stops wrist motors
    public void wristStop() {
        wristMotor1.set(0);
        blinkin.chaseRed();
        // wristMotor2.set(0);
    }

    public void zeroEncoders() {
        // Zero Out Encoder Positions
        wristEncoder1.setPosition(0.0);
        System.err.println("Zeroed Intake Encoders");
    }

    // Wrist up movement control
    public void wristUp() {
        if (getWristLimit()) {
            wristStop();
        } else {
            //wristMotor1.set(-.2);
            wristMotor1.set(wristPID.calculate(wristEncoder1.getPosition(), 1.0));
        }
    }

    public void wristHalf() {
        if (getWristLimit()) {
            wristStop();
        } else {
            wristMotor1.set(wristPID.calculate(wristEncoder1.getPosition(), 18));
        }
    }

    // Wrist down movement control
    public void wristDown() {
        wristMotor1.set(wristPID.calculate(wristEncoder1.getPosition(), 36.2));
    }

    // Intake speed set
    public void intakeStart(double speed) {
        intakeMotor.set(speed);
        blinkin.rainbowRGB();
    }

    // Returns Instance Of Intake
    public static Intake getInstance() {
        if (m_Instance == null) {
            m_Instance = new Intake(Constants.Intake.wristMotor1ID, Constants.Intake.intakeMotorID);
        }

        return m_Instance;
    }

}
