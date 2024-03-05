package frc.robot.Subsystems.Vision;
import edu.wpi.first.wpilibj.Servo;

/**
 * Subsystem for the mechanism that keeps the balls from moving around.
 */
public class pivot{
	private static pivot instance = null;
	private static final boolean invertLeft = true;
	private static final boolean invertRight = false;
	
	//These need to match the implementation of angle ranges in the Servo class.
	//  There's no good reason to not have this supported.
	private static final double MAX_ANGLE_LEFT = 170.0;
	private static final double MAX_ANGLE_RIGHT = 170.0;
	private static final double MIN_ANGLE_LEFT = 0.0;
	private static final double MIN_ANGLE_RIGHT = 0.0;




//setting the angle
private void setAngle(double angle, boolean invert, Servo servo,
			double minAngle, double maxAngle) {

		//The Servo class has hard coded ranges of travel.
		if(minAngle < 0) {
			minAngle = 0.0;
		}
		
		if(maxAngle > 170.0){
			maxAngle = 170.0;
		}
		
		//Get angle in range
		if(angle > maxAngle) {
			angle = maxAngle;
		} else if(angle < minAngle) {
			angle = minAngle;
		}
		
		if(invert) {
			angle = maxAngle - angle;
		}
		servo.setAngle(angle);
    }

	/**
	 * A private constructor to prevent multiple instances from being created.
	 */

}