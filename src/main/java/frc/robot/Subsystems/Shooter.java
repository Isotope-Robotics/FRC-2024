package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter {

    public static CANSparkMax shooterMotor1;
    public static CANSparkMax shooterMotor2;
    public static CANSparkMax pivotMotor;
    public static RelativeEncoder shooter1Encoder;
    public static RelativeEncoder shooter2Encoder;
    public static RelativeEncoder pivotEncoder;
    public static DigitalInput noteDetected;
    public static DigitalOutput relay;
	


    private static Shooter m_Instance = null;

    public static final PIDController pivotPID = new PIDController(Constants.Shooter.kP2, Constants.Shooter.kI2,
            Constants.Shooter.kD2);

    private final Blinkin blinkin = Blinkin.getInstance();


    public Shooter(int shooter1CANID, int shooter2CANID, int pivotMotorID) {
        // Motor Declarations
        shooterMotor1 = new CANSparkMax(shooter1CANID, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(shooter2CANID, MotorType.kBrushless);
        pivotMotor = new CANSparkMax(pivotMotorID, MotorType.kBrushless);


        // Idle Mode Declarations
        shooterMotor1.setIdleMode(Constants.Shooter.Brake);
        shooterMotor2.setIdleMode(Constants.Shooter.Brake);
        pivotMotor.setIdleMode(Constants.Shooter.Brake);

        // Set Direction of the Motors
        shooterMotor1.setInverted(false);
        shooterMotor2.setInverted(false);
        pivotMotor.setInverted(false);

        // Encoders Declarations
        shooter1Encoder = shooterMotor1.getEncoder(SparkRelativeEncoder.Type.kHallSensor, Constants.Encoders.NEO_ENCODER_COUNTS);
        shooter2Encoder = shooterMotor2.getEncoder(SparkRelativeEncoder.Type.kHallSensor, Constants.Encoders.NEO_ENCODER_COUNTS);
        pivotEncoder = pivotMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, Constants.Encoders.NEO_ENCODER_COUNTS);
        //noteDetected = new DigitalInput(1);
        relay = new DigitalOutput(8);
    }

    public void lightsOn() {
        relay.set(true);
        System.out.println("lights onn!!");
       }

       public void lightsOff() {
      
        relay.set(false);
                System.out.println("lights offfffffffff!!");

       }

    // Set both shooter motors to shoot (adjust the speed as needed)
    public void shoot(double speed) {
        shooterMotor1.set(-1.0f*speed);
        shooterMotor2.set(speed);
        SmartDashboard.putNumber("Shooter 1 Speed", speed);
        SmartDashboard.putNumber("Shooter 2 Speed", speed);
    }

    public void shoot2(double speed) {
        shooterMotor1.set(-speed);
        shooterMotor2.set(speed - 0.2);
        SmartDashboard.putNumber("Shooter 1 Speed", speed);
        SmartDashboard.putNumber("Shooter 2 Speed", speed);
    }

    // public boolean getNoteDetected() {
    //     if (!noteDetected.get()) {
    //        // blinkin.rainbowRGB();
    //         return true;
    //     } else {
    //         return false;
    //     }
    // }

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
        pivotEncoder.setPosition(0);
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

    //Clear Sticky Faults
    public void clearStickyFaults(){
        shooterMotor1.clearFaults();
        shooterMotor2.clearFaults();
        pivotMotor.clearFaults();
        System.out.println("Clearing Shooter Faults, If Any");
    }

    public static void pivotDown() {
      // pivotMotor.setPosition(-1);
      SmartDashboard.putNumber("pivot", pivotEncoder.getPosition());
    }

    public static void pivotUp() {
      //  pivotMotor.setPosition(0);

    }

    public double getPivotCurrent() {
        return pivotMotor.getOutputCurrent();
    }

    public  double getShooter1Current() {
        return shooterMotor1.getOutputCurrent();
    }

    public  double getShooter2Current() {
        return shooterMotor2.getOutputCurrent();
    }

    // Returns Instance Of Shooter
    public static Shooter getInstance() {
        if (m_Instance == null) {
            m_Instance = new Shooter(Constants.Shooter.shooterMotor1ID, Constants.Shooter.shooterMotor2ID, Constants.Shooter.pivotMotorID);
        }
        return m_Instance;
    }
    //setting the angle
    
        }
