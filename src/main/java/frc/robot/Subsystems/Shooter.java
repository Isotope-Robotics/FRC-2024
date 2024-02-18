package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder;

import frc.robot.Constants;

public class Shooter {

    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    public static RelativeEncoder shooter1Encoder;
    public static RelativeEncoder shooter2Encoder;
    public static DigitalInput noteDetected;

    private static Shooter m_Instance = null;

    public Shooter(int shooter1CANID, int shooter2CANID) {
        // Motor Declarations
        shooterMotor1 = new CANSparkMax(shooter1CANID, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(shooter2CANID, MotorType.kBrushless);

        // Idle Mode Declarations
        shooterMotor1.setIdleMode(Constants.Shooter.Brake);
        shooterMotor2.setIdleMode(Constants.Shooter.Brake);

        // Set Direction of the Motors
        shooterMotor1.setInverted(false);
        shooterMotor2.setInverted(false);

        // Encoders Declarations
        shooter1Encoder = shooterMotor1.getEncoder(SparkRelativeEncoder.Type.kHallSensor, Constants.Encoders.NEO_ENCODER_COUNTS);
        shooter2Encoder = shooterMotor2.getEncoder(SparkRelativeEncoder.Type.kHallSensor, Constants.Encoders.NEO_ENCODER_COUNTS);
        noteDetected = new DigitalInput(1);
    }

    // Set both shooter motors to shoot (adjust the speed as needed)
    public void shoot(double speed) {
        shooterMotor1.set(-speed);
        shooterMotor2.set(speed);
        SmartDashboard.putNumber("Shooter 1 Speed", speed);
        SmartDashboard.putNumber("Shooter 2 Speed", speed);
    }

    public boolean getNoteDetected() {
        if (!noteDetected.get()) {
            return true;
        } else {
            return false;
        }
    }

    // Stop both shooter motors
    public void stop() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
        SmartDashboard.putNumber("Shooter is Stopped", 0);
    }

    public void zeroEncoders() {
        // Zero Out Encoder Positions
        shooter1Encoder.setPosition(0);
        shooter2Encoder.setPosition(0);
        System.err.println("Zeroed Shooter Encoders");
    }

    // Set both shooter motors to intake through the source (adjust the speed as
    // needed)

    public void sourceIntake(double motor1Speed, double motor2Speed) {
        shooterMotor1.set(-motor1Speed);
        shooterMotor2.set(-motor2Speed);
        SmartDashboard.putNumber("Shooter 1 Speed", motor1Speed);
        SmartDashboard.putNumber("Shooter 2 Speed", motor2Speed);

    }

    // Returns Instance Of Shooter
    public static Shooter getInstance() {
        if (m_Instance == null) {
            m_Instance = new Shooter(Constants.Shooter.shooterMotor1ID, Constants.Shooter.shooterMotor2ID);
        }
        return m_Instance;
    }
}
