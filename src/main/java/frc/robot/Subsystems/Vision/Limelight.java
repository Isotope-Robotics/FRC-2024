package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Limelight {
    private static NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry ts;

    public PIDController pidController = new PIDController(0.1, 0, 0.001);

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
    }

    // Read values periodically
    public void updateLimelightData() {
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        // Post to smart dashboard periodically
        // SmartDashboard.putNumber("LimelightX", x);
        // SmartDashboard.putNumber("LimelightY", y);
        // SmartDashboard.putNumber("LimelightArea", area);
        
    }

    public double limelight_aim_proportional() {
        double kP = 0.01;
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

        targetingAngularVelocity *= Constants.Swerve.maxAngularVelocity;

        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    double limelight_range_proportional()
    {    
      double kP = .1;
      double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
      targetingForwardSpeed *= Constants.Swerve.maxSpeed;
      targetingForwardSpeed *= -1.0;
      return targetingForwardSpeed;
    }

    public static  void ledOn(boolean isOn) {
        if (isOn) {
        table.getEntry("ledMode").setNumber(3);

        } else {
                    table.getEntry("ledMode").setNumber(1);

        }
    } 

}
